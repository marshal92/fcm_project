#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

namespace fcm_plugins
{
class SaveCurrentPoseToBlackboard : public BT::SyncActionNode
{
public:
  SaveCurrentPoseToBlackboard(const std::string & name, const BT::NodeConfig & conf)
  : BT::SyncActionNode(name, conf)
  {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose_key")
    };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped current_pose;
    try {
      // Узнаем, где находится робот (base_link) относительно карты (map)
      auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      current_pose.header = transform.header;
      current_pose.pose.position.x = transform.transform.translation.x;
      current_pose.pose.position.y = transform.transform.translation.y;
      current_pose.pose.orientation = transform.transform.rotation;
      
      // Записываем в Blackboard
      setOutput("pose_key", current_pose);
      return BT::NodeStatus::SUCCESS;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "Сбой TF: %s", ex.what());
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};
} // ВАЖНО: Закрываем namespace ЗДЕСЬ

#include "behaviortree_cpp/bt_factory.h"

// Принудительный экспорт функции для ROS 2
extern "C" __attribute__((visibility("default"))) void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<fcm_plugins::SaveCurrentPoseToBlackboard>("SaveCurrentPoseToBlackboard");
}