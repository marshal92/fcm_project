#include "fcm_costmap_plugins/radiation_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(fcm_costmap_plugins::RadiationLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;

namespace fcm_costmap_plugins
{

RadiationLayer::RadiationLayer()
: need_bounds_update_(false)
{
}

void RadiationLayer::onInitialize()
{
  auto node = node_.lock(); 
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }

  declareParameter("radiation_topic", rclcpp::ParameterValue("/radiation_map"));
  
  // === ПАРАМЕТРЫ ИЗ ФОРМУЛЫ (2.41) ===
  declareParameter("r_noise", rclcpp::ParameterValue(15.0)); 
  declareParameter("r_crit", rclcpp::ParameterValue(90.0)); 
  declareParameter("r_soft", rclcpp::ParameterValue(15.0)); 
  declareParameter("beta", rclcpp::ParameterValue(1.7)); 
  declareParameter("c_lethal", rclcpp::ParameterValue(252)); 

  std::string topic = node->get_parameter(name_ + ".radiation_topic").as_string();
  
  // === ИСПОЛЬЗУЕМ TRANSIENT LOCAL QoS ДЛЯ СОВМЕСТИМОСТИ ===
  rad_map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic, rclcpp::QoS(1).transient_local(),
    std::bind(&RadiationLayer::mapCallback, this, std::placeholders::_1));

  current_ = true;
  enabled_ = true;
  
  RCLCPP_INFO(node->get_logger(), "===> RadiationLayer V3 (Formula 2.41 + Transient Local) Initialized!");
}

void RadiationLayer::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_rad_map_ = msg;
  need_bounds_update_ = true;
  
  auto node = node_.lock();
  if (node) {
      RCLCPP_INFO_ONCE(node->get_logger(), "===> BINGO! First radiation map received by C++ plugin!");
  }
}

void RadiationLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_ || !latest_rad_map_) return;

  std::lock_guard<std::mutex> lock(data_mutex_);
  if (need_bounds_update_) {
    double map_min_x = latest_rad_map_->info.origin.position.x;
    double map_min_y = latest_rad_map_->info.origin.position.y;
    double map_max_x = map_min_x + (latest_rad_map_->info.width * latest_rad_map_->info.resolution);
    double map_max_y = map_min_y + (latest_rad_map_->info.height * latest_rad_map_->info.resolution);

    *min_x = std::min(*min_x, map_min_x);
    *min_y = std::min(*min_y, map_min_y);
    *max_x = std::max(*max_x, map_max_x);
    *max_y = std::max(*max_y, map_max_y);
    
    need_bounds_update_ = false;
  }
}

void RadiationLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !latest_rad_map_) return;

  std::lock_guard<std::mutex> lock(data_mutex_);
  auto node = node_.lock();
  
  // Диагностика: выводит сообщение раз в 10 секунд во время работы
  auto & clk = *node->get_clock();
  RCLCPP_INFO_THROTTLE(node->get_logger(), clk, 10000, "===> Applying ALARA costs to master grid...");
  
  double r_noise = node->get_parameter(name_ + ".r_noise").as_double();
  double r_crit  = node->get_parameter(name_ + ".r_crit").as_double();
  double r_soft  = node->get_parameter(name_ + ".r_soft").as_double();
  double beta    = node->get_parameter(name_ + ".beta").as_double();
  int c_lethal   = node->get_parameter(name_ + ".c_lethal").as_int();
  
  if (c_lethal > 252) c_lethal = 252;

  double res = latest_rad_map_->info.resolution;
  double ox = latest_rad_map_->info.origin.position.x;
  double oy = latest_rad_map_->info.origin.position.y;
  int width = latest_rad_map_->info.width;
  int height = latest_rad_map_->info.height;

  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);
      
      int rad_x = static_cast<int>((wx - ox) / res);
      int rad_y = static_cast<int>((wy - oy) / res);

      if (rad_x < 0 || rad_x >= width || rad_y < 0 || rad_y >= height) continue;

      int rad_index = rad_y * width + rad_x;
      int8_t rad_value = latest_rad_map_->data[rad_index];

      // Пропускаем "неизвестные" зоны из карты телеметрии (-1)
      if (rad_value < 0) continue;

      double R = static_cast<double>(rad_value);
      int cost = 0;

      // ==========================================================
      // РЕАЛИЗАЦИЯ ФОРМУЛЫ (2.41) ИЗ ДИССЕРТАЦИИ
      // ==========================================================
      if (R <= r_noise) {
        cost = 0;
      } 
      else if (R >= r_crit) {
        cost = c_lethal;
      } 
      else {
        double exponent = (R - r_noise) / r_soft;
        double raw_cost = beta * (std::exp(exponent) - 1.0);
        cost = static_cast<int>(raw_cost);
      }
      // ==========================================================

      cost = std::min(cost, c_lethal);
      cost = std::max(cost, 0);

      unsigned char old_cost = master_grid.getCost(i, j);
      
      // ВАЖНО: Разрешаем наложение поверх серой зоны (NO_INFORMATION)
      // Блокируем перезапись только реальных физических препятствий (стен)
      if (old_cost == LETHAL_OBSTACLE) {
        continue; 
      }
      
      master_grid.setCost(i, j, std::max(static_cast<int>(old_cost), cost));
    }
  }
}

void RadiationLayer::reset()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_rad_map_.reset();
  need_bounds_update_ = true;
}

}  // namespace fcm_costmap_plugins