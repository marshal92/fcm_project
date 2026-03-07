#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import tf2_ros
import time
import csv
import math

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

        # Открываем CSV файл для записи (сохранится в папке, откуда запущен скрипт)
        self.csv_file = open('mission_telemetry.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time_sec', 'X_m', 'Y_m', 'Speed_mps', 'Dose_Rate_uSv_h', 'Accum_Dose_uSv', 'Distance_m'])

        map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = self.create_subscription(OccupancyGrid, '/radiation_map', self.map_callback, map_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.log_telemetry)
        self.print_timer = self.create_timer(2.0, self.print_stats)

        self.get_logger().info("Telemetry Logger started! Writing to mission_telemetry.csv...")

    def map_callback(self, msg):
        self.latest_map = msg

    def odom_callback(self, msg):
        # Читаем текущую физическую скорость танка из одометрии
        self.current_speed = msg.twist.twist.linear.x

    def log_telemetry(self):
        if self.latest_map is None: return

        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx = trans.transform.translation.x
            ry = trans.transform.translation.y
        except Exception: return

        # Считаем пройденный путь
        if self.last_x is not None and self.last_y is not None:
            dx = rx - self.last_x
            dy = ry - self.last_y
            self.total_distance += math.hypot(dx, dy)
        self.last_x = rx
        self.last_y = ry

        # Ищем радиацию под танком
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

        # Интегрируем дозу
        self.accumulated_dose += dose_rate * self.dt_hours

        # Пишем строку в CSV
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
            f"Speed: {self.current_speed:.2f}m/s | Total Dose: {self.accumulated_dose:.2f} µSv"
        )

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_file.close() # Обязательно закрываем файл при выходе
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()