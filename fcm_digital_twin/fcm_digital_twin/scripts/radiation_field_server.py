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

        # 1. ЖЕСТКИЕ ИСТОЧНИКИ (Куски ТСМ)
        # size - физический диаметр куска (в метрах), создает "плато" максимальной дозы
        self.hard_splatters = [
            {'x': 1.5,  'y': 2.0,  'intensity': 50000.0, 'size': 0.3},
            {'x': -3.0, 'y': 0.0,  'intensity': 80000.0, 'size': 0.5},
            {'x': 6.5,  'y': 7.5,  'intensity': 40000.0, 'size': 0.25}
        ]

        # 2. МЯГКИЕ ИСТОЧНИКИ (Радиоактивная пыль / Аэрозоли)
        # sigma - размытость облака. Они будут складываться друг с другом!
        self.soft_clouds = [
            {'x': 0.0, 'y': 5.0, 'intensity': 3000.0, 'sigma': 2.0}, 
            {'x': 1.0, 'y': 4.0, 'intensity': 2500.0, 'sigma': 1.5}, # Пересекается с первым!
            {'x': 4.0, 'y': 1.0, 'intensity': 1500.0, 'sigma': 1.5}  
        ]

        map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.rad_pub = self.create_publisher(OccupancyGrid, '/radiation_map', map_qos)
        self.get_logger().info("Realistic Radiation Server with Splatters & Gaussians started!")

    def map_callback(self, map_msg):
        width = map_msg.info.width
        height = map_msg.info.height
        res = map_msg.info.resolution
        ox = map_msg.info.origin.position.x
        oy = map_msg.info.origin.position.y

        x = np.linspace(0, width - 1, width) * res + ox
        y = np.linspace(0, height - 1, height) * res + oy
        xv, yv = np.meshgrid(x, y)

        total_dose = np.zeros((height, width), dtype=np.float32)

        # --- СЧИТАЕМ ЖЕСТКИЕ ПЯТНА (Кляксы с плато) ---
        for src in self.hard_splatters:
            dx = xv - src['x']
            dy = yv - src['y']
            dist = np.sqrt(dx**2 + dy**2)
            
            # Процедурная деформация круга в неровную "кляксу"
            angle = np.arctan2(dy, dx)
            # Фаза зависит от координат, чтобы все кляксы были разной формы
            phase = src['x'] * 10.0 
            noise = 1.0 + 0.3 * np.sin(3 * angle + phase) + 0.15 * np.cos(5 * angle - phase)
            
            # Искаженное расстояние
            dist_irregular = dist * noise
            
            # Радиус плато (физический размер куска)
            core_r = src['size'] / 2.0
            
            # Формула с плато: I / (r^2 + core_r^2) * exp(-mu * r)
            dose = (src['intensity'] / (dist_irregular**2 + core_r**2)) * np.exp(-self.mu_air * dist_irregular)
            total_dose += dose

        # --- СЧИТАЕМ МЯГКИЕ ОБЛАКА (Гауссианы) ---
        for cloud in self.soft_clouds:
            dist_sq = (xv - cloud['x'])**2 + (yv - cloud['y'])**2
            # Гауссово распределение (красиво и физично складывается при пересечении)
            dose = cloud['intensity'] * np.exp(-dist_sq / (2 * cloud['sigma']**2))
            total_dose += dose

        # --- МАСШТАБИРОВАНИЕ ---
        scaled_field = (total_dose / self.max_lethal_dose) * 100.0
        
        # Легчайший базовый фон здания (чтобы зоны вне пыли не были абсолютным нулем)
        # Если RViz будет красить все в розовый, поставь здесь 0.0
        scaled_field += 1.0 
        
        scaled_field = np.clip(np.round(scaled_field), 0, 100).astype(np.int8)

        # Маска SLAM
        slam_map = np.array(map_msg.data).reshape((height, width))
        scaled_field[slam_map == -1] = -1

        rad_msg = OccupancyGrid()
        rad_msg.header = map_msg.header
        rad_msg.info = map_msg.info
        rad_msg.data = scaled_field.flatten().tolist()
        self.rad_pub.publish(rad_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RadiationFieldServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()