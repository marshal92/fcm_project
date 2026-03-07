#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import math
from ament_index_python.packages import get_package_share_directory

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from action_msgs.msg import GoalStatus

class ShadowTeleop(Node):
    def __init__(self):
        super().__init__('shadow_teleop')
        
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel_shadow', self.cmd_cb, 10)
        self.command_sub = self.create_subscription(String, '/shadow_command', self.command_cb, 10)
        self.external_goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.external_goal_cb, 10)
        
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_cb, 1)
        self.current_costmap = None
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, '/shadow_marker', 10)
        self.path_pub = self.create_publisher(Path, '/shadow_path', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        
        self.current_goal_handle = None 
        
        pkg_share_dir = get_package_share_directory('fcm_digital_twin')
        self.mesh_uri = f"file://{pkg_share_dir}/meshes/shadow.STL"
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        
        self.recorded_poses = []
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
        self.record_distance = 0.5 
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_pose_valid = False
        
        self.state = 'IDLE' 
        self.was_moving = False 
        
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / 30.0, self.update_loop)
        self.get_logger().info("Shadow Node v11 [0-100 Costmap Fix & Strict Walls] Started!")

    def get_yaw_from_quat(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def cmd_cb(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def costmap_cb(self, msg):
        self.current_costmap = msg

    def get_costmap_value(self, x, y):
        """Возвращает сырое значение (0-100) из OccupancyGrid для точки x, y"""
        if self.current_costmap is None:
            return 0 
            
        res = self.current_costmap.info.resolution
        origin_x = self.current_costmap.info.origin.position.x
        origin_y = self.current_costmap.info.origin.position.y
        width = self.current_costmap.info.width
        height = self.current_costmap.info.height
        
        grid_x = int((x - origin_x) / res)
        grid_y = int((y - origin_y) / res)
        
        if grid_x < 0 or grid_x >= width or grid_y < 0 or grid_y >= height:
            return -1 # За пределами карты
            
        index = grid_y * width + grid_x
        return self.current_costmap.data[index]

    def check_path_clear(self, x0, y0, x1, y1):
        """Трассировка луча (Raycasting) для защиты от пролета сквозь тонкие стены"""
        dist = math.hypot(x1 - x0, y1 - y0)
        if dist == 0:
            return self.get_costmap_value(x1, y1)
            
        res = self.current_costmap.info.resolution if self.current_costmap else 0.05
        step = res / 2.0  # Шаг проверки - полпикселя костмапа
        
        steps = int(dist / step)
        if steps == 0:
            return self.get_costmap_value(x1, y1)
            
        dx = (x1 - x0) / steps
        dy = (y1 - y0) / steps
        
        max_cost = 0
        for i in range(1, steps + 1):
            cx = x0 + dx * i
            cy = y0 + dy * i
            c = self.get_costmap_value(cx, cy)
            
            if c == -1: return -1
            if c > max_cost: max_cost = c
            
            # В OccupancyGrid 99 = Inscribed (граница робота касается препятствия)
            # 100 = Lethal (центр робота в препятствии)
            if max_cost >= 99:
                return max_cost
                
        return max_cost

    def update_robot_pose(self):
        try:
            now = rclpy.time.Time()
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', now)
            self.robot_x = t.transform.translation.x
            self.robot_y = t.transform.translation.y
            self.robot_yaw = self.get_yaw_from_quat(t.transform.rotation)
            self.robot_pose_valid = True
        except Exception:
            self.robot_pose_valid = False

    def external_goal_cb(self, msg):
        dist = math.hypot(self.x - msg.pose.position.x, self.y - msg.pose.position.y)
        if dist > 0.1:
            self.get_logger().info("RViz Клик! Телепортирую тень...")
            self.x = msg.pose.position.x
            self.y = msg.pose.position.y
            self.theta = self.get_yaw_from_quat(msg.pose.orientation)
            
            self.recorded_poses.clear()
            self.path_msg.poses.clear()
            self.state = 'IDLE'
            self.was_moving = False

    def force_append_current_pose(self):
        if len(self.recorded_poses) > 0:
            last_pose = self.recorded_poses[-1].pose.position
            dist = math.hypot(self.x - last_pose.x, self.y - last_pose.y)
            if dist > 0.02: 
                qz = math.sin(self.theta / 2.0)
                qw = math.cos(self.theta / 2.0)
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'map'
                pose.pose.position.x = self.x
                pose.pose.position.y = self.y
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                self.recorded_poses.append(pose)
                self.path_msg.poses = self.recorded_poses
                self.path_pub.publish(self.path_msg)

    def cancel_active_goal(self):
        if self.current_goal_handle is not None:
            self.get_logger().warn(">>> ОТМЕНА ЗАДАЧИ NAV2 <<<")
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None

    def command_cb(self, msg):
        cmd = msg.data.lower()
        
        if cmd in ['stop', 'abort']:
            self.cancel_active_goal()
            self.state = 'IDLE'
            
        elif cmd == 'clear':
            self.get_logger().info("КНОПКА ПАНИКИ: Сброс маршрута и возврат Тени к танку.")
            self.cancel_active_goal()
            self.recorded_poses.clear()
            self.path_msg.poses.clear()
            if self.robot_pose_valid:
                self.x = self.robot_x
                self.y = self.robot_y
                self.theta = self.robot_yaw
            self.state = 'IDLE'
            self.was_moving = False
            
        elif cmd == 'execute':
            if len(self.recorded_poses) > 0:
                self.force_append_current_pose()
                self.send_nav_path()

    def send_nav_path(self):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            return
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self.recorded_poses
        self.state = 'NAVIGATING'
        self.get_logger().info(f"Старт маршрута ({len(self.recorded_poses)} точек)")
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.state = 'IDLE'
            return
        self.current_goal_handle = goal_handle 
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Миссия выполнена успешно!")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Миссия прервана оператором.")
        
        self.current_goal_handle = None
        self.state = 'IDLE'
        self.recorded_poses.clear()
        self.path_msg.poses.clear()

    def update_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        
        self.update_robot_pose()
        
        # --- ФИЗИКА ТЕНИ С ТРАССИРОВКОЙ ЛУЧЕЙ (CCD) ---
        new_x = self.x + self.v * math.cos(self.theta) * dt
        new_y = self.y + self.v * math.sin(self.theta) * dt
        new_theta = self.theta + self.w * dt
        
        path_cost = self.check_path_clear(self.x, self.y, new_x, new_y)
        target_cost = self.get_costmap_value(new_x, new_y)
        current_cost = self.get_costmap_value(self.x, self.y)
        
        CRITICAL_COST = 99 # В OccupancyGrid 99 = граница бампера
        WARNING_COST = 5   # Радиация ALARA (все что выше 5 будет оранжевым)
        
        status = 'SAFE'
        
        if path_cost >= CRITICAL_COST or path_cost == -1:
            status = 'CRITICAL'
            
            safe_target_cost = 101 if target_cost == -1 else target_cost
            safe_current_cost = 101 if current_cost == -1 else current_cost
            
            # Разрешаем "выползти" из стены, если движение уменьшает опасность
            if safe_target_cost < safe_current_cost:
                self.x = new_x
                self.y = new_y
            
            self.theta = new_theta # Разрешаем крутиться на месте
        else:
            self.x = new_x
            self.y = new_y
            self.theta = new_theta
            if path_cost >= WARNING_COST:
                status = 'WARNING'

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'shadow_base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)
        
        # --- ЦВЕТОВАЯ ИНДИКАЦИЯ ---
        marker = Marker()
        marker.header.frame_id = 'shadow_base_link'
        marker.header.stamp = now.to_msg()
        marker.ns = 'shadow_robot'
        marker.type = Marker.MESH_RESOURCE
        
        # Используем динамический путь вместо хардкода!
        marker.mesh_resource = self.mesh_uri 
        marker.action = Marker.ADD
        marker.scale.x = 0.001
        marker.scale.y = 0.001
        marker.scale.z = 0.001
        
        dist_to_robot = math.hypot(self.x - self.robot_x, self.y - self.robot_y)
        HIDE_THRESHOLD = 0.35 
        
        # Если робот слишком близко - прячем под землю
        if self.robot_pose_valid and dist_to_robot < HIDE_THRESHOLD:
            marker.pose.position.z = -10.0  # Отправляем в преисподнюю
            marker.color.a = 0.0
            marker.pose.orientation.w = 1.0
        else:
            marker.pose.position.z = 0.0    # Возвращаем на поверхность
            marker.pose.orientation.w = 1.0
            
            if status == 'CRITICAL':
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 0.0, 0.7 
            elif status == 'WARNING':
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.5, 0.0, 0.7 
            else:
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 1.0, 0.4 
            
        self.marker_pub.publish(marker)
                                
        # --- ЛОГИКА ЗАПИСИ ПУТИ ---
        is_moving = abs(self.v) > 0.01 or abs(self.w) > 0.01
        
        if is_moving:
            self.was_moving = True 
            
            if self.state in ['IDLE', 'DRAWING']:
                if self.state == 'IDLE' and self.robot_pose_valid:
                    self.state = 'DRAWING'
                    self.recorded_poses.clear()
                    start_pose = PoseStamped()
                    start_pose.header.stamp = now.to_msg()
                    start_pose.header.frame_id = 'map'
                    start_pose.pose.position.x = self.robot_x
                    start_pose.pose.position.y = self.robot_y
                    start_pose.pose.orientation.z = math.sin(self.robot_yaw / 2.0)
                    start_pose.pose.orientation.w = math.cos(self.robot_yaw / 2.0)
                    self.recorded_poses.append(start_pose)

                if len(self.recorded_poses) > 0:
                    last_pose = self.recorded_poses[-1].pose.position
                    dist_to_last = math.hypot(self.x - last_pose.x, self.y - last_pose.y)
                    if dist_to_last >= self.record_distance:
                        pose = PoseStamped()
                        pose.header.stamp = now.to_msg()
                        pose.header.frame_id = 'map'
                        pose.pose.position.x = self.x
                        pose.pose.position.y = self.y
                        pose.pose.orientation.z = qz
                        pose.pose.orientation.w = qw
                        self.recorded_poses.append(pose)
                        self.path_msg.poses = self.recorded_poses
        else:
            if self.was_moving and self.state == 'DRAWING':
                self.force_append_current_pose()
                self.was_moving = False
                
        if len(self.recorded_poses) > 0:
            self.path_pub.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ShadowTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()