#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class RadiationFieldServer(Node):
    def __init__(self):
        super().__init__('radiation_field_server')

        self.mu_air = 0.9  
        self.max_lethal_dose = 15000.0 

        self.hard_splatters = [
            {'x': 10.6,  'y': 9.0,  'intensity': 85000.0, 'size': 1.45},
            {'x': 1.5, 'y': 1.5,  'intensity': 50000.0, 'size': 0.7},
            {'x': 7.9,  'y': 16.5,  'intensity': 65000.0, 'size': 1.0},
            {'x': 11.5,  'y': 14.5,  'intensity': 45000.0, 'size': 0.9},
            {'x': 5.0,  'y': 9.3,  'intensity': 50000.0, 'size': 0.75},
            {'x': 7.5,  'y': 0.3,  'intensity': 45000.0, 'size': 0.5},
            {'x': 2.2,  'y': -2.6,  'intensity': 60000.0, 'size': 0.8}
        ]

        self.soft_clouds = [
            {'x': -1.0,  'y': 6.0,  'intensity': 4000.0, 'sigma': 1.8},
            {'x': 3.0, 'y': 0.0, 'intensity': 3000.0, 'sigma': 2.0}, 
            {'x': 8.0, 'y': 8.0, 'intensity': 2500.0, 'sigma': 1.5},
            {'x': 5.0, 'y': 20.0, 'intensity': 1500.0, 'sigma': 1.5},
            {'x': 14.0,  'y': 19.0,  'intensity': 4000.0, 'sigma': 1.8},
            {'x': 12.0, 'y': 0.0, 'intensity': 3000.0, 'sigma': 2.0} 
        ]

        map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.rad_pub = self.create_publisher(OccupancyGrid, '/radiation_map', map_qos)
        self.declare_parameter('is_active', False)
        
        self.cached_width = 0
        self.cached_height = 0
        self.cached_total_dose = None
        
        # --- OPTIMIZATION: Publication Management  ---
        self.last_map_msg = None
        self.last_published_state = None 
        self.need_publish = False
        self.pub_timer = self.create_timer(0.5, self.publish_loop)

        self.get_logger().info("Super-Optimized Radiation Server started!")

    def map_callback(self, map_msg):
        self.last_map_msg = map_msg
        width = map_msg.info.width
        height = map_msg.info.height
        res = map_msg.info.resolution
        ox = map_msg.info.origin.position.x
        oy = map_msg.info.origin.position.y

        if width != self.cached_width or height != self.cached_height:
            self.get_logger().info(f"Map size changed to {width}x{height}. Recalculating radiation field...")
            
            x = np.linspace(0, width - 1, width) * res + ox
            y = np.linspace(0, height - 1, height) * res + oy
            xv, yv = np.meshgrid(x, y)

            total_dose = np.zeros((height, width), dtype=np.float32)

            for src in self.hard_splatters:
                dx = xv - src['x']
                dy = yv - src['y']
                dist = np.sqrt(dx**2 + dy**2)
                angle = np.arctan2(dy, dx)
                
                f1 = 2.0 + (src['x'] % 5.0)  
                f2 = 2.0 + (src['y'] % 5.0)
                a1 = 0.1 + (src['y'] % 0.2)  
                a2 = 0.05 + (src['x'] % 0.15)
                
                noise = 1.0 + a1 * np.sin(f1 * angle + src['x']) + a2 * np.cos(f2 * angle + src['y'])
                r_effective = dist * noise
                r_core = src['size']
                
                dose = (src['intensity'] / ((r_effective / r_core)**2 + 1.0)) * np.exp(-self.mu_air * r_effective)
                total_dose += dose
            
            for cloud in self.soft_clouds:
                dist_sq = (xv - cloud['x'])**2 + (yv - cloud['y'])**2
                dose = cloud['intensity'] * np.exp(-dist_sq / (2 * cloud['sigma']**2))
                total_dose += dose

            self.cached_total_dose = total_dose
            self.cached_width = width
            self.cached_height = height
            self.need_publish = True 

    def publish_loop(self):
        if self.last_map_msg is None or self.cached_total_dose is None:
            return

        is_active = self.get_parameter('is_active').get_parameter_value().bool_value
        
        # Publish only if the radiation status (ON/OFF) has changed or the map size has been updated
        if is_active != self.last_published_state or self.need_publish:
            
            width = self.cached_width
            height = self.cached_height
            
            if is_active:
                scaled_field = (self.cached_total_dose / self.max_lethal_dose) * 100.0
                scaled_field += 1.0 
                scaled_field = np.clip(np.round(scaled_field), 0, 100).astype(np.int8)
            else:
                scaled_field = np.ones((height, width), dtype=np.int8)

            slam_map = np.array(self.last_map_msg.data).reshape((height, width))
            scaled_field[slam_map == -1] = -1

            rad_msg = OccupancyGrid()
            rad_msg.header = self.last_map_msg.header
            rad_msg.info = self.last_map_msg.info
            rad_msg.data = scaled_field.flatten().tolist()
            
            self.rad_pub.publish(rad_msg)
            
            self.last_published_state = is_active
            self.need_publish = False
            self.get_logger().info(f"Radiation map updated and published. State: {'ON' if is_active else 'OFF'}")

def main(args=None):
    rclpy.init(args=args)
    node = RadiationFieldServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()