#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math

class PrecisionAligner(Node):
    def __init__(self):
        super().__init__('precision_aligner')
        
        # 1. Слушаем финальную цель от нашей Тени
        self.goal_sub = self.create_subscription(
            PoseStamped, '/precision_goal', self.goal_cb, 10)
            
        # 2. Слушаем текущее положение робота от одометрии
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)
            
        # 3. Публикуем скорости напрямую на моторы (с высоким приоритетом)
        # В идеале это топик для twist_mux, но пока шлем в стандартный cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.active = False
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Настройки точности (допуски)
        self.xy_tol = 0.02   # 2 сантиметра!
        self.yaw_tol = 0.02  # ~1 градус!
        
        # Состояния конечного автомата: TURN_TO_GOAL -> DRIVE_TO_GOAL -> ALIGN_YAW -> DONE
        self.state = 'DONE'
        
        self.timer = self.create_timer(0.05, self.control_loop) # 20 Гц
        self.get_logger().info("Precision Aligner Node Started. Waiting for final goals.")

    def get_yaw_from_quat(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        """Нормализация угла в пределы [-pi, pi]"""
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.get_yaw_from_quat(msg.pose.pose.orientation)

    def goal_cb(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_yaw = self.get_yaw_from_quat(msg.pose.orientation)
        
        self.active = True
        self.state = 'TURN_TO_GOAL'
        self.get_logger().info("Отримано ціль для точної доводки! Починаю паркування.")

    def control_loop(self):
        if not self.active:
            return
            
        cmd = Twist()
        
        # Дистанция до цели
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        distance = math.hypot(dx, dy)
        
        # Угол направления на цель
        angle_to_goal = math.atan2(dy, dx)
        
        if self.state == 'TURN_TO_GOAL':
            error_yaw = self.normalize_angle(angle_to_goal - self.current_yaw)
            if distance < self.xy_tol:
                self.state = 'ALIGN_YAW' # Если уже близко, просто доворачиваем корпус
            elif abs(error_yaw) > self.yaw_tol:
                cmd.angular.z = 1.0 * error_yaw # P-регулятор поворота
                # Ограничиваем скорость
                cmd.angular.z = max(min(cmd.angular.z, 0.5), -0.5)
            else:
                self.state = 'DRIVE_TO_GOAL'
                
        elif self.state == 'DRIVE_TO_GOAL':
            error_yaw = self.normalize_angle(angle_to_goal - self.current_yaw)
            if distance > self.xy_tol:
                cmd.linear.x = 0.5 * distance # P-регулятор скорости
                cmd.linear.x = max(min(cmd.linear.x, 0.2), 0.05) # Едем медленно и аккуратно
                cmd.angular.z = 0.5 * error_yaw # Подруливаем
            else:
                self.state = 'ALIGN_YAW'
                
        elif self.state == 'ALIGN_YAW':
            error_yaw = self.normalize_angle(self.goal_yaw - self.current_yaw)
            if abs(error_yaw) > self.yaw_tol:
                cmd.angular.z = 1.0 * error_yaw
                cmd.angular.z = max(min(cmd.angular.z, 0.5), -0.5)
            else:
                self.get_logger().info("Паркування успішно завершено!")
                self.active = False
                self.state = 'DONE'
                
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PrecisionAligner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()