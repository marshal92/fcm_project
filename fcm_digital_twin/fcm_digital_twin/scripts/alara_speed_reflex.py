#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import SpeedLimit
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import tf2_ros

class AlaraSpeedReflex(Node):
    def __init__(self):
        super().__init__('alara_speed_reflex')

        self.rad_threshold = 20.0        
        
        # Задаем скорости в ПРОЦЕНТАХ от максимума в navigation.yaml
        self.normal_speed_pct = 35.0  # Спокойная езда (35% от 1.2 м/с = ~0.4 м/с)
        self.boost_speed_pct = 100.0  # ГАЗ В ПОЛ! (100% от 1.2 м/с)

        self.is_boosted = False
        self.latest_map = None

        map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = self.create_subscription(OccupancyGrid, '/radiation_map', self.map_callback, map_qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ОФИЦИАЛЬНЫЙ ТОПИК NAV2 ДЛЯ ОГРАНИЧЕНИЯ СКОРОСТИ
        self.limit_pub = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        
        self.timer_check = self.create_timer(0.1, self.check_reflex)
        self.get_logger().info("ALARA Native Speed Limit Reflex started!")

    def map_callback(self, msg):
        self.latest_map = msg

    def publish_speed_limit(self, pct):
        msg = SpeedLimit()
        msg.percentage = True     # Говорим Nav2, что это проценты, а не м/с
        msg.speed_limit = pct
        self.limit_pub.publish(msg)

    def check_reflex(self):
        if self.latest_map is None: return

        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx = trans.transform.translation.x
            ry = trans.transform.translation.y
        except Exception: return

        res = self.latest_map.info.resolution
        ox = self.latest_map.info.origin.position.x
        oy = self.latest_map.info.origin.position.y
        width = self.latest_map.info.width

        rad_x = int((rx - ox) / res)
        rad_y = int((ry - oy) / res)

        if rad_x < 0 or rad_y < 0 or rad_x >= width or rad_y >= self.latest_map.info.height: return
        rad_val = self.latest_map.data[rad_y * width + rad_x]

        # Логика переключения
        if rad_val >= self.rad_threshold and not self.is_boosted:
            self.get_logger().warn(f"RADIATION ({rad_val}%)! BOOSTING TO {self.boost_speed_pct}% SPEED!")
            self.is_boosted = True
            
        elif rad_val < self.rad_threshold and self.is_boosted:
            self.get_logger().info(f"Radiation cleared. Restoring limit to {self.normal_speed_pct}%.")
            self.is_boosted = False

        # Постоянно шлем лимит, чтобы Nav2 держал его (требование Nav2)
        if self.is_boosted:
            self.publish_speed_limit(self.boost_speed_pct)
        else:
            self.publish_speed_limit(self.normal_speed_pct)

def main(args=None):
    rclpy.init(args=args)
    node = AlaraSpeedReflex()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()