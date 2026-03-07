#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace fcm_plugins
{
class SetBasePose : public BT::SyncActionNode
{
public:
  SetBasePose(const std::string & name, const BT::NodeConfig & conf)
  : BT::SyncActionNode(name, conf) {}

  static BT::PortsList providedPorts() {
    return { BT::OutputPort<geometry_msgs::msg::PoseStamped>("base_pose") };
  }

  BT::NodeStatus tick() override {
    geometry_msgs::msg::PoseStamped base;
    base.header.frame_id = "map";
    // Координаты базы (можете потом поменять на нужные)
    base.pose.position.x = 0.0;
    base.pose.position.y = 0.0;
    base.pose.orientation.w = 1.0;
    
    setOutput("base_pose", base); // Кладем в общую память дерева
    return BT::NodeStatus::SUCCESS;
  }
};
} 

#include "behaviortree_cpp/bt_factory.h"
extern "C" __attribute__((visibility("default"))) void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<fcm_plugins::SetBasePose>("SetBasePose");
}