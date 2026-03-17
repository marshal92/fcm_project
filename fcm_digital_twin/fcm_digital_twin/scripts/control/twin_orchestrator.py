#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TwinOrchestrator(Node):
    def __init__(self):
        super().__init__('twin_orchestrator')
        
        # Підписуємось на команди від кнопок із Foxglove
        # Foxglove буде публікувати рядки сюди
        self.sub = self.create_subscription(String, '/operator_command', self.command_cb, 10)
        
        # Створюємо паблішер для прямого управління Тінню (замість subprocess)
        self.shadow_pub = self.create_publisher(String, '/shadow_command', 10)
        
        self.get_logger().info("Серверний узел Twin Orchestrator запущен!")
        self.get_logger().info("Ожидание кнопок из Foxglove в топике /operator_command...")

    def command_cb(self, msg):
        cmd = msg.data.lower()
        
        if cmd == 'execute':
            self.get_logger().info("🚀 Отримано команду EXECUTE від Foxglove. Запуск Тіні...")
            # Нативно публікуємо в топік /shadow_command
            shadow_msg = String()
            shadow_msg.data = 'execute'
            self.shadow_pub.publish(shadow_msg)
            
        elif cmd == 'clear':
            self.get_logger().warn("🛑 Отримано команду clear (clear) від Foxglove!")
            # Нативно публікуємо в топік /shadow_command
            shadow_msg = String()
            shadow_msg.data = 'clear'
            self.shadow_pub.publish(shadow_msg)
            
        else:
            self.get_logger().error(f"Невідома команда від Foxglove: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = TwinOrchestrator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Остановка Twin Orchestrator...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()