#include <string>
#include <chrono>
#include <thread>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "std_msgs/msg/empty.hpp"

namespace fcm_plugins
{
class IsConnectionLost : public BT::ConditionNode
{
public:
  IsConnectionLost(const std::string & condition_name, const BT::NodeConfig & conf)
  : BT::ConditionNode(condition_name, conf), timeout_(3.0), first_message_received_(false)
  {
    auto main_node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    getInput("timeout", timeout_);

    RCLCPP_INFO(main_node->get_logger(), "\033[1;32m[Heartbeat] Автономный слушатель запущен. Таймаут: %.1f сек\033[0m", timeout_);

    // 1. Создаем эксклюзивную группу коллбеков, чтобы Nav2 ее не блокировал
    callback_group_ = main_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;

    // 2. Подписываемся на пульс
    sub_ = main_node->create_subscription<std_msgs::msg::Empty>(
      "/operator_heartbeat", 10,
      [this, main_node](const std_msgs::msg::Empty::SharedPtr) {
        last_heartbeat_time_ = std::chrono::steady_clock::now();
        if (!first_message_received_) {
            RCLCPP_INFO(main_node->get_logger(), "\033[1;34m[Heartbeat] ПЕРВЫЙ ПУЛЬС ПОЛУЧЕН! Движение разрешено.\033[0m");
            first_message_received_ = true;
        }
      }, sub_options);

    // 3. ЗАПУСКАЕМ ОТДЕЛЬНЫЙ ПОТОК (Гарантия получения сообщений)
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(callback_group_, main_node->get_node_base_interface());
    
    spin_thread_ = std::thread([this]() {
      executor_->spin();
    });
  }

  // Деструктор: аккуратно убиваем поток при выключении робота
  ~IsConnectionLost() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("timeout", 3.0, "Таймаут") };
  }

  BT::NodeStatus tick() override
  {
    auto main_node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    // ИСПРАВЛЕННАЯ ЛОГИКА: Если пульса ЕЩЕ НЕ БЫЛО - робот не имеет права ехать!
    if (!first_message_received_) {
      return BT::NodeStatus::SUCCESS; // Возвращаем SUCCESS (Условие "СВЯЗЬ ПОТЕРЯНА" = Истина)
    }

    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - last_heartbeat_time_;

    if (elapsed.count() > timeout_) {
      RCLCPP_WARN(main_node->get_logger(), "\033[1;31m[Heartbeat] ОБРЫВ СВЯЗИ! Тормозим... (Нет сигнала %.1f сек)\033[0m", elapsed.count());
      first_message_received_ = false; // Сбрасываем флаг, чтобы ждать восстановления
      return BT::NodeStatus::SUCCESS;  // Связь потеряна
    }

    return BT::NodeStatus::FAILURE; // Связь ЕСТЬ (Условие "Связь потеряна" = Ложь)
  }

private:
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  
  std::chrono::time_point<std::chrono::steady_clock> last_heartbeat_time_;
  double timeout_;
  bool first_message_received_;
};
} // namespace fcm_plugins

#include "behaviortree_cpp/bt_factory.h"
extern "C" __attribute__((visibility("default"))) void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<fcm_plugins::IsConnectionLost>("IsConnectionLost");
}