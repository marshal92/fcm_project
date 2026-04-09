#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TwinOrchestrator(Node):
    def __init__(self):
        super().__init__('twin_orchestrator')
        
        # Subscribe to commands from buttons with Foxglove
        # Foxglove will publish strings here
        self.sub = self.create_subscription(String, '/operator_command', self.command_cb, 10)
        
        # Create a publisher for direct control of the Shadow (instead of subprocess)
        self.shadow_pub = self.create_publisher(String, '/shadow_command', 10)
        
        self.get_logger().info("Server node Twin Orchestrator started!")
        self.get_logger().info("Waiting for buttons from Foxglove in topic /operator_command...")

    def command_cb(self, msg):
        cmd = msg.data.lower()
        
        if cmd == 'execute':
            self.get_logger().info("Received EXECUTE command from Foxglove. Starting Shadow...")
            # Natively publish to the /shadow_command topic
            shadow_msg = String()
            shadow_msg.data = 'execute'
            self.shadow_pub.publish(shadow_msg)
            
        elif cmd == 'clear':
            self.get_logger().warn("🛑 Received CLEAR command from Foxglove!")
            # Natively publish to the /shadow_command topic
            shadow_msg = String()
            shadow_msg.data = 'clear'
            self.shadow_pub.publish(shadow_msg)
            
        else:
            self.get_logger().error(f"Unknown command from Foxglove: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = TwinOrchestrator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping Twin Orchestrator...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()