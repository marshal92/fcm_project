#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from collections import deque

class LatencySimulator(Node):
    def __init__(self):
        super().__init__('latency_simulator')
        
        self.declare_parameter('input_topic', '/cmd_vel_in')
        self.declare_parameter('output_topic', '/cmd_vel_out')
        self.declare_parameter('delay_ms', 1000) 
        
        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.delay_sec = self.get_parameter('delay_ms').value / 1000.0
        
        self.msg_queue = deque()
        
        self.sub = self.create_subscription(Twist, in_topic, self.callback, 10)
        self.pub = self.create_publisher(Twist, out_topic, 10)
        
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.get_logger().info(
            f"🚀 Latency Simulator запущен!\n"
            f"Вход: {in_topic} -> Выход: {out_topic}\n"
            f"Штучная задержка: {self.delay_sec} сек."
        )

    def callback(self, msg):
        self.msg_queue.append((time.time(), msg))

    def timer_callback(self):
        current_time = time.time()
        while self.msg_queue and (current_time - self.msg_queue[0][0]) >= self.delay_sec:
            _, msg = self.msg_queue.popleft()
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LatencySimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()