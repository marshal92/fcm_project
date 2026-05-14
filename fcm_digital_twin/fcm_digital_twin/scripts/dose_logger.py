#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import tf2_ros
import time
import csv
import math
import os
from datetime import datetime

class TelemetryLogger(Node):
    def __init__(self):
        super().__init__('telemetry_logger')

        self.max_lethal_dose = 15000.0  
        self.update_rate_hz = 10.0      
        self.dt_hours = (1.0 / self.update_rate_hz) / 3600.0  

        self.latest_map = None
        self.accumulated_dose = 0.0     
        self.total_distance = 0.0
        self.current_speed = 0.0
        self.last_x = None
        self.last_y = None
        self.start_time = time.time()

        # --- ГЕНЕРАЦИЯ УНИКАЛЬНОГО ИМЕНИ ФАЙЛА ---
        # 1. Создаем папку для логов в домашней директории пользователя
        log_dir = os.path.expanduser('~/fcm_mission_logs')
        os.makedirs(log_dir, exist_ok=True)
        
        # 2. Генерируем таймстемп (ГодМесяцДень_ЧасМинутСекунды)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'mission_{timestamp}.csv'
        
        # 3. Формируем полный путь
        self.filepath = os.path.join(log_dir, filename)

        self.csv_file = open(self.filepath, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time_sec', 'X_m', 'Y_m', 'Speed_mps', 'Dose_Rate_uSv_h', 'Accum_Dose_uSv', 'Distance_m'])
        
        self.get_logger().info(f"✅ Логгер запущен! Файл сохраняется в: {self.filepath}")
        # ----------------------------------------

        map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = self.create_subscription(OccupancyGrid, '/radiation_map', self.map_cb, map_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.timer_cb)
        self.print_timer = self.create_timer(5.0, self.print_stats)

    def map_cb(self, msg):
        self.latest_map = msg

    def odom_cb(self, msg):
        self.current_speed = msg.twist.twist.linear.x

    def timer_cb(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx = trans.transform.translation.x
            ry = trans.transform.translation.y
        except Exception:
            return

        if self.last_x is not None and self.last_y is not None:
            dist = math.hypot(rx - self.last_x, ry - self.last_y)
            self.total_distance += dist

        self.last_x = rx
        self.last_y = ry

        if self.latest_map is None:
            return

        res = self.latest_map.info.resolution
        ox = self.latest_map.info.origin.position.x
        oy = self.latest_map.info.origin.position.y
        width = self.latest_map.info.width

        rad_x = int((rx - ox) / res)
        rad_y = int((ry - oy) / res)

        dose_rate = 0.0
        if 0 <= rad_x < width and 0 <= rad_y < self.latest_map.info.height:
            rad_percent = self.latest_map.data[rad_y * width + rad_x]
            if rad_percent > 0:
                dose_rate = (rad_percent / 100.0) * self.max_lethal_dose

        self.accumulated_dose += dose_rate * self.dt_hours

        elapsed_time = time.time() - self.start_time
        self.csv_writer.writerow([
            round(elapsed_time, 2), 
            round(rx, 3), round(ry, 3), 
            round(self.current_speed, 3), 
            round(dose_rate, 1), 
            round(self.accumulated_dose, 3), 
            round(self.total_distance, 3)
        ])

    def print_stats(self):
        elapsed_time = time.time() - self.start_time
        self.get_logger().info(
            f"T: {elapsed_time:.1f}s | Dist: {self.total_distance:.2f}m | "
            f"Speed: {self.current_speed:.2f}m/s | Dose: {self.accumulated_dose:.1f}µSv"
        )

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Логгер остановлен оператором.")
    finally:
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()