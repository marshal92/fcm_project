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
: source_x_(0.0), source_y_(0.0), source_intensity_(0.0), need_update_(false)
{
}

void RadiationLayer::onInitialize()
{
  auto node = node_.lock(); 
  if (!node) {
    throw std::runtime_error("Failed to lock node");
  }

  declareParameter("radiation_topic", rclcpp::ParameterValue("/radiation_sensor"));
  declareParameter("influence_radius", rclcpp::ParameterValue(1.0));
  declareParameter("max_radiation_limit", rclcpp::ParameterValue(500.0));

  std::string topic = node->get_parameter(name_ + ".radiation_topic").as_string();
  
  radiation_sub_ = node->create_subscription<geometry_msgs::msg::Point32>(
    topic, 10,
    std::bind(&RadiationLayer::radiationCallback, this, std::placeholders::_1));

  current_ = true;
  enabled_ = true;
  
  RCLCPP_INFO(node->get_logger(), "Digital Twin RadiationLayer initialized!");
}

void RadiationLayer::radiationCallback(const geometry_msgs::msg::Point32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  source_x_ = msg->x;
  source_y_ = msg->y;
  source_intensity_ = msg->z;
  need_update_ = true;
}

void RadiationLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  (void) robot_yaw; (void) robot_x; (void) robot_y;
  if (!enabled_ || !need_update_) return;

  std::lock_guard<std::mutex> lock(data_mutex_);
  auto node = node_.lock();
  double influence_radius = node->get_parameter(name_ + ".influence_radius").as_double();

  // Обновляем костмап ВОКРУГ источника радиации, а не вокруг робота
  *min_x = std::min(*min_x, source_x_ - influence_radius);
  *min_y = std::min(*min_y, source_y_ - influence_radius);
  *max_x = std::max(*max_x, source_x_ + influence_radius);
  *max_y = std::max(*max_y, source_y_ + influence_radius);
}

void RadiationLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !need_update_) return;

  std::lock_guard<std::mutex> lock(data_mutex_);
  if (source_intensity_ <= 5.0) return; 

  auto node = node_.lock();
  double influence_radius = node->get_parameter(name_ + ".influence_radius").as_double();
  double max_rad_limit = node->get_parameter(name_ + ".max_radiation_limit").as_double();

  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);
      
      double distance = std::hypot(wx - source_x_, wy - source_y_);
      
      if (distance <= influence_radius) {
        double cell_radiation = source_intensity_ * (1.0 - (distance / influence_radius));
        if (cell_radiation < 0) cell_radiation = 0;
        
        // ОГРАНИЧЕНИЕ 252 (Non-lethal), чтобы избежать паралича
        int cost = static_cast<int>((cell_radiation / max_rad_limit) * 252.0);
        if (cost > 252) cost = 252;
        
        unsigned char old_cost = master_grid.getCost(i, j);
        if (old_cost == LETHAL_OBSTACLE) continue;
        
        master_grid.setCost(i, j, std::max((int)old_cost, cost));
      }
    }
  }
}

void RadiationLayer::reset()
{
  source_intensity_ = 0.0;
  need_update_ = false;
}

}  // namespace fcm_costmap_plugins