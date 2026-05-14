#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import tf2_ros
import time
import csv
import math
import os
from datetime import datetime

class ShadowEvaluator(Node):
    def __init__(self):
        super().__init__('shadow_evaluator')

        self.update_rate_hz = 10.0      
        self.dt = 1.0 / self.update_rate_hz
        
        # Данные телеметрии
        self.current_speed = 0.0
        self.last_speed = 0.0
        self.last_accel = 0.0
        self.min_clearance = float('inf')
        
        # Координаты
        self.rx = 0.0
        self.ry = 0.0
        
        # Фильтр (Moving Average)
        self.speed_buffer = []
        self.buffer_size = 5 
        
        self.start_time = time.time()

        # Файловая система
        log_dir = os.path.expanduser('~/fcm_mission_logs')
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filepath = os.path.join(log_dir, f'shadow_eval_{timestamp}.csv')

        self.csv_file = open(self.filepath, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # Добавлены X_m и Y_m
        self.csv_writer.writerow(['Time_sec', 'X_m', 'Y_m', 'Speed_mps', 'Accel_mps2', 'Jerk_mps3', 'Clearance_m'])
        
        self.get_logger().info(f"✅ Shadow Evaluator (TF + Filter) запущен! Файл: {self.filepath}")

        # Подписки
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, QoSProfile(depth=5, durability=QoSDurabilityPolicy.VOLATILE))

        # TF2 для координат
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(self.dt, self.timer_cb)
        self.print_timer = self.create_timer(2.0, self.print_stats)

    def odom_cb(self, msg):
        self.current_speed = msg.twist.twist.linear.x

    def scan_cb(self, msg):
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max and not math.isinf(r) and not math.isnan(r)]
        if valid_ranges:
            self.min_clearance = min(valid_ranges)
        else:
            self.min_clearance = float('inf')

    def timer_cb(self):
        # 1. Получаем координаты через TF
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.rx = trans.transform.translation.x
            self.ry = trans.transform.translation.y
        except Exception:
            # Если трансформация еще не готова, просто пропускаем итерацию
            return

        # 2. Фильтрация скорости
        self.speed_buffer.append(self.current_speed)
        if len(self.speed_buffer) > self.buffer_size:
            self.speed_buffer.pop(0)
            
        smoothed_speed = sum(self.speed_buffer) / len(self.speed_buffer)

        # 3. Расчет динамики
        accel = (smoothed_speed - self.last_speed) / self.dt
        jerk = (accel - self.last_accel) / self.dt
        
        self.last_speed = smoothed_speed
        self.last_accel = accel

        elapsed_time = time.time() - self.start_time
        clearance_val = self.min_clearance if not math.isinf(self.min_clearance) else 0.0

        self.csv_writer.writerow([
            round(elapsed_time, 2), 
            round(self.rx, 3), 
            round(self.ry, 3), 
            round(smoothed_speed, 3), 
            round(accel, 3), 
            round(jerk, 3), 
            round(clearance_val, 3)
        ])

    def print_stats(self):
        elapsed_time = time.time() - self.start_time
        self.get_logger().info(
            f"T: {elapsed_time:.1f}s | Pos: ({self.rx:.1f}, {self.ry:.1f}) | Jerk: {self.last_accel:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ShadowEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Логгер остановлен.")
    finally:
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()