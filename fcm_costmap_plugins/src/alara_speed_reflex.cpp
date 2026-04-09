#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class AlaraSpeedReflex : public rclcpp::Node
{
public:
  AlaraSpeedReflex()
  : Node("alara_speed_reflex_cpp"),
    rad_threshold_(30.0),
    normal_speed_pct_(35.0),
    boost_speed_pct_(100.0),
    is_boosted_(false),
    init_done_(false)
  {
    // Transient Local QoS
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/radiation_map", qos, std::bind(&AlaraSpeedReflex::map_callback, this, _1));

    limit_pub_ = this->create_publisher<nav2_msgs::msg::SpeedLimit>("/speed_limit", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_check_ = this->create_wall_timer(
      500ms, std::bind(&AlaraSpeedReflex::check_reflex, this));

    timer_init_ = this->create_wall_timer(
      1000ms, std::bind(&AlaraSpeedReflex::init_speed, this));

    RCLCPP_INFO(this->get_logger(), "ALARA Native C++ Reflex started! (Zero CPU load)");
  }

private:
  void init_speed()
  {
    if (!init_done_) {
      publish_speed_limit(normal_speed_pct_);
      init_done_ = true;
    }
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    latest_map_ = msg;
  }

  void publish_speed_limit(double pct)
  {
    nav2_msgs::msg::SpeedLimit msg;
    msg.percentage = true;
    msg.speed_limit = pct;
    limit_pub_->publish(msg);
  }

  void check_reflex()
  {
    if (!latest_map_) return;

    geometry_msgs::msg::TransformStamped trans;
    try {
      trans = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      return; 
    }

    double rx = trans.transform.translation.x;
    double ry = trans.transform.translation.y;

    double res = latest_map_->info.resolution;
    double ox = latest_map_->info.origin.position.x;
    double oy = latest_map_->info.origin.position.y;
    int width = latest_map_->info.width;
    int height = latest_map_->info.height;

    int rad_x = static_cast<int>((rx - ox) / res);
    int rad_y = static_cast<int>((ry - oy) / res);

    if (rad_x < 0 || rad_y < 0 || rad_x >= width || rad_y >= height) return;

    int8_t rad_val = latest_map_->data[rad_y * width + rad_x];

    if (rad_val >= rad_threshold_ && !is_boosted_) {
      RCLCPP_WARN(this->get_logger(), "RADIATION (%d%%)! BOOSTING TO %.0f%% SPEED!", rad_val, boost_speed_pct_);
      is_boosted_ = true;
      publish_speed_limit(boost_speed_pct_);
    } else if (rad_val < rad_threshold_ && is_boosted_) {
      RCLCPP_INFO(this->get_logger(), "Radiation cleared. Restoring limit to %.0f%%.", normal_speed_pct_);
      is_boosted_ = false;
      publish_speed_limit(normal_speed_pct_);
    }
  }

  double rad_threshold_, normal_speed_pct_, boost_speed_pct_;
  bool is_boosted_, init_done_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr limit_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_check_;
  rclcpp::TimerBase::SharedPtr timer_init_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AlaraSpeedReflex>());
  rclcpp::shutdown();
  return 0;
}