#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from action_msgs.msg import GoalStatus
import math
import numpy as np
from ament_index_python.packages import get_package_share_directory


class ShadowTeleopReal(Node):
    def __init__(self):
        super().__init__('shadow_teleop')

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel_shadow', self.cmd_cb, 10)
        self.command_sub = self.create_subscription(
            String, '/shadow_command', self.command_cb, 10)
        self.external_goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.external_goal_cb, 10)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.costmap_cb, 1)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, '/shadow_marker', 10)
        self.path_pub   = self.create_publisher(Path,   '/shadow_path',   10)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client  = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')

        self.current_goal_handle = None

        pkg = get_package_share_directory('fcm_digital_twin')
        self.mesh_uri = f"file://{pkg}/meshes/shadow.STL"

        self.x = self.y = self.theta = self.v = self.w = 0.0
        self.robot_x = self.robot_y = self.robot_yaw = 0.0
        self.robot_pose_valid = False

        self.recorded_poses: list[PoseStamped] = []
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
        self.record_distance = 0.5
        self._path_dirty = False

        self.state      = 'IDLE'
        self.was_moving = False

        # Costmap в numpy — конвертируем один раз при получении
        self._costmap_array: np.ndarray | None = None
        self._costmap_info = None

        # Переиспользуемые объекты — не создаём каждый тик
        self._tf_msg = TransformStamped()
        self._tf_msg.header.frame_id   = 'map'
        self._tf_msg.child_frame_id    = 'shadow_base_link'
        self._tf_msg.transform.translation.z = 0.0
        self._tf_msg.transform.rotation.x    = 0.0
        self._tf_msg.transform.rotation.y    = 0.0

        self._marker = Marker()
        self._marker.ns           = 'shadow_robot'
        self._marker.type         = Marker.MESH_RESOURCE
        self._marker.mesh_resource = self.mesh_uri
        self._marker.action       = Marker.ADD
        self._marker.scale.x = self._marker.scale.y = self._marker.scale.z = 0.001
        self._marker.pose.orientation.w = 1.0

        self._ui_counter = 0
        self.last_time   = self.get_clock().now()

        # На реальном роботе use_sim_time=false → таймер использует OS-время, всё ок
        self.timer = self.create_timer(1.0 / 30.0, self.update_loop)

        self.get_logger().info("Shadow Node Started (REAL ROBOT)")

    # ------------------------------------------------------------------ #
    #  Costmap                                                             #
    # ------------------------------------------------------------------ #

    def costmap_cb(self, msg: OccupancyGrid):
        """Конвертируем в numpy один раз — все lookup O(1)."""
        self._costmap_info  = msg.info
        self._costmap_array = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)

    def get_costmap_value(self, x: float, y: float) -> int:
        if self._costmap_array is None:
            return 0
        info = self._costmap_info
        gx = int((x - info.origin.position.x) / info.resolution)
        gy = int((y - info.origin.position.y) / info.resolution)
        h, w = self._costmap_array.shape
        if not (0 <= gx < w and 0 <= gy < h):
            return -1
        return int(self._costmap_array[gy, gx])

    def check_path_clear(self, x0, y0, x1, y1) -> int:
        """Векторизованный raycast — без Python-цикла."""
        if self._costmap_array is None:
            return 0
        dist = math.hypot(x1 - x0, y1 - y0)
        if dist == 0:
            return self.get_costmap_value(x1, y1)
        steps = max(1, int(dist / (self._costmap_info.resolution / 2.0)))
        t   = np.linspace(0.0, 1.0, steps + 1)[1:]
        xs  = x0 + (x1 - x0) * t
        ys  = y0 + (y1 - y0) * t
        info = self._costmap_info
        gxs = ((xs - info.origin.position.x) / info.resolution).astype(int)
        gys = ((ys - info.origin.position.y) / info.resolution).astype(int)
        h, w = self._costmap_array.shape
        if not np.all((gxs >= 0) & (gxs < w) & (gys >= 0) & (gys < h)):
            return -1
        return int(np.max(self._costmap_array[gys, gxs]))

    # ------------------------------------------------------------------ #
    #  Колбеки                                                             #
    # ------------------------------------------------------------------ #

    def cmd_cb(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def external_goal_cb(self, msg: PoseStamped):
        if math.hypot(self.x - msg.pose.position.x, self.y - msg.pose.position.y) > 0.1:
            self.x, self.y = msg.pose.position.x, msg.pose.position.y
            self.theta = self._yaw(msg.pose.orientation)
            self.recorded_poses.clear()
            self.path_msg.poses.clear()
            self._path_dirty = True
            self.state, self.was_moving = 'IDLE', False

    def update_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            self.robot_x   = t.transform.translation.x
            self.robot_y   = t.transform.translation.y
            self.robot_yaw = self._yaw(t.transform.rotation)
            self.robot_pose_valid = True
        except Exception:
            self.robot_pose_valid = False

    # ------------------------------------------------------------------ #
    #  Навигация — нормальные методы вместо lambda-цепочки                #
    # ------------------------------------------------------------------ #

    def cancel_active_goal(self):
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None

    def command_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd in ('stop', 'abort'):
            self.cancel_active_goal()
            self.state = 'IDLE'
        elif cmd == 'clear':
            self.cancel_active_goal()
            self.recorded_poses.clear()
            self.path_msg.poses.clear()
            self._path_dirty = True
            if self.robot_pose_valid:
                self.x, self.y, self.theta = self.robot_x, self.robot_y, self.robot_yaw
            self.state, self.was_moving = 'IDLE', False
        elif cmd == 'execute' and self.recorded_poses:
            self._force_append_current_pose()
            self._send_nav_path()

    def _send_nav_path(self):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 недоступен!")
            return
        goal = NavigateThroughPoses.Goal()
        goal.poses = self.recorded_poses
        self.state = 'NAVIGATING'
        self.get_logger().info(f"Старт маршрута ({len(self.recorded_poses)} точек)")
        self.nav_client.send_goal_async(goal).add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Цель отклонена Nav2.")
            self.state = 'IDLE'
            return
        self.current_goal_handle = handle
        handle.get_result_async().add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Миссия выполнена.")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Миссия отменена.")
        else:
            self.get_logger().error(f"Миссия завершилась со статусом: {status}")
        self.current_goal_handle = None
        self.state = 'IDLE'
        self.recorded_poses.clear()
        self.path_msg.poses.clear()
        self._path_dirty = True

    # ------------------------------------------------------------------ #
    #  Вспомогательные                                                     #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _yaw(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _make_pose(self, x, y, theta, stamp) -> PoseStamped:
        p = PoseStamped()
        p.header.stamp    = stamp
        p.header.frame_id = 'map'
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.z = math.sin(theta / 2.0)
        p.pose.orientation.w = math.cos(theta / 2.0)
        return p

    def _force_append_current_pose(self):
        if not self.recorded_poses:
            return
        last = self.recorded_poses[-1].pose.position
        if math.hypot(self.x - last.x, self.y - last.y) > 0.02:
            self.recorded_poses.append(
                self._make_pose(self.x, self.y, self.theta, self.get_clock().now().to_msg()))
            self.path_msg.poses = self.recorded_poses
            self._path_dirty = True

    # ------------------------------------------------------------------ #
    #  Главный цикл                                                        #
    # ------------------------------------------------------------------ #

    def update_loop(self):
        now     = self.get_clock().now()
        dt      = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        stamp   = now.to_msg()

        self.update_robot_pose()

        # --- Физика ---
        new_x     = self.x + self.v * math.cos(self.theta) * dt
        new_y     = self.y + self.v * math.sin(self.theta) * dt
        new_theta = self.theta + self.w * dt

        path_cost = self.check_path_clear(self.x, self.y, new_x, new_y)
        status = 'SAFE'

        if path_cost >= 99 or path_cost == -1:
            status = 'CRITICAL'
            tc = self.get_costmap_value(new_x, new_y)
            cc = self.get_costmap_value(self.x,  self.y)
            if (101 if tc == -1 else tc) < (101 if cc == -1 else cc):
                self.x, self.y = new_x, new_y
            self.theta = new_theta
        else:
            self.x, self.y, self.theta = new_x, new_y, new_theta
            if path_cost >= 5:
                status = 'WARNING'

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # --- TF (переиспользуем объект) ---
        self._tf_msg.header.stamp           = stamp
        self._tf_msg.transform.translation.x = self.x
        self._tf_msg.transform.translation.y = self.y
        self._tf_msg.transform.rotation.z   = qz
        self._tf_msg.transform.rotation.w   = qw
        self.tf_broadcaster.sendTransform(self._tf_msg)

        # --- Визуализация 10Hz ---
        self._ui_counter += 1
        if self._ui_counter < 3:
            if self._path_dirty and self.recorded_poses:
                self.path_pub.publish(self.path_msg)
                self._path_dirty = False
            return
        self._ui_counter = 0

        is_moving  = abs(self.v) > 0.01 or abs(self.w) > 0.01
        hide       = (self.robot_pose_valid
                      and math.hypot(self.x - self.robot_x, self.y - self.robot_y) < 0.35
                      and not is_moving)

        m = self._marker
        m.header.stamp    = stamp
        m.header.frame_id = 'shadow_base_link'
        if hide:
            m.pose.position.z = -10.0
            m.color.a = 0.0
        else:
            m.pose.position.z = 0.0
            if status == 'CRITICAL':
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.7
            elif status == 'WARNING':
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 0.7
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 1.0, 0.4
        self.marker_pub.publish(m)

        # --- Запись пути ---
        if is_moving:
            self.was_moving = True
            if self.state in ('IDLE', 'DRAWING'):
                if self.state == 'IDLE' and self.robot_pose_valid:
                    self.state = 'DRAWING'
                    self.recorded_poses.clear()
                    self.recorded_poses.append(
                        self._make_pose(self.robot_x, self.robot_y, self.robot_yaw, stamp))
                if self.recorded_poses:
                    last = self.recorded_poses[-1].pose.position
                    if math.hypot(self.x - last.x, self.y - last.y) >= self.record_distance:
                        self.recorded_poses.append(
                            self._make_pose(self.x, self.y, self.theta, stamp))
                        self.path_msg.poses = self.recorded_poses
                        self._path_dirty = True
        else:
            if self.was_moving and self.state == 'DRAWING':
                self._force_append_current_pose()
                self.was_moving = False

        if self._path_dirty and self.recorded_poses:
            self.path_pub.publish(self.path_msg)
            self._path_dirty = False


def main(args=None):
    rclpy.init(args=args)
    node = ShadowTeleopReal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()