#!/usr/bin/env python3
import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from action_msgs.msg import GoalStatus
import math, threading, time
import numpy as np
from ament_index_python.packages import get_package_share_directory

class ShadowTeleopSim(Node):
    def __init__(self):
        super().__init__('shadow_teleop')

        self._running = True

        # --- QoS ---
        cmd_qos   = QoSProfile(depth=5)
        costmap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # === ВАЖНО: НИКАКИХ ПОДПИСОК НА /clock! ===

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel_shadow', self.cmd_cb, cmd_qos)
        self.command_sub = self.create_subscription(String, '/shadow_command', self.command_cb, cmd_qos)
        self.external_goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.external_goal_cb, cmd_qos)
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.costmap_cb, costmap_qos)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub  = self.create_publisher(Marker, '/shadow_marker', 10)
        self.path_pub    = self.create_publisher(Path,   '/shadow_path',   5)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.current_goal_handle = None

        pkg = get_package_share_directory('fcm_digital_twin')
        self.mesh_uri = f"file://{pkg}/meshes/shadow.STL"

        self.x = self.y = self.theta = self.v = self.w = 0.0
        self.recorded_poses: list[PoseStamped] = []
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
        self.record_distance = 0.5
        self._path_dirty = False

        self.robot_x = self.robot_y = self.robot_yaw = 0.0
        self.robot_pose_valid = False
        self.state = 'IDLE'
        self.was_moving = False

        self._costmap_array: np.ndarray | None = None
        self._costmap_info = None

        self._tf_msg = TransformStamped()
        self._tf_msg.header.frame_id = 'map'
        self._tf_msg.child_frame_id  = 'shadow_base_link'
        self._tf_msg.transform.translation.z = 0.0
        self._tf_msg.transform.rotation.x    = 0.0
        self._tf_msg.transform.rotation.y    = 0.0

        self._marker = Marker()
        self._marker.ns   = 'shadow_robot'
        self._marker.type = Marker.MESH_RESOURCE
        self._marker.mesh_resource = self.mesh_uri
        self._marker.action = Marker.ADD
        self._marker.scale.x = self._marker.scale.y = self._marker.scale.z = 0.001
        self._marker.pose.orientation.w = 1.0

        self._ui_counter = 0
        self._last_real_ns: int | None = None   

        # Переменная для "украденного" времени
        self.latest_sim_time_msg = None

        self._loop_thread = threading.Thread(
            target=self._real_time_loop, daemon=True, name='shadow_loop'
        )
        self._loop_thread.start()

        self.get_logger().info("Shadow Node SIMULATION Started (Time Thief Mode - No CPU overload!)")

    # ------------------------------------------------------------------ #
    #  OS-поток: настоящий 30Hz на системных часах                       #
    # ------------------------------------------------------------------ #

    def _real_time_loop(self):
        period = 1.0 / 30.0
        while self._running and rclpy.ok():
            t_start = time.monotonic()
            try:
                self._update()
            except Exception as e:
                self.get_logger().error(f"update_loop error: {e}", throttle_duration_sec=5.0)
            elapsed = time.monotonic() - t_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    # ------------------------------------------------------------------ #
    #  Колбеки подписок                                                  #
    # ------------------------------------------------------------------ #

    def cmd_cb(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def costmap_cb(self, msg: OccupancyGrid):
        self._costmap_info  = msg.info
        self._costmap_array = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)

    def external_goal_cb(self, msg: PoseStamped):
        dist = math.hypot(self.x - msg.pose.position.x, self.y - msg.pose.position.y)
        if dist > 0.1:
            self.x     = msg.pose.position.x
            self.y     = msg.pose.position.y
            self.theta = self._yaw_from_quat(msg.pose.orientation)
            self.recorded_poses.clear()
            self.path_msg.poses.clear()
            self._path_dirty = True
            self.state = 'IDLE'
            self.was_moving = False

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

    @staticmethod
    def _yaw_from_quat(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def cancel_active_goal(self):
        if self.current_goal_handle is not None:
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
            self.state = 'IDLE'
            self.was_moving = False
        elif cmd == 'execute':
            if self.recorded_poses:
                self._force_append_current_pose()
                self._send_nav_path()

    def _make_pose_stamped(self, x, y, theta) -> PoseStamped:
        p = PoseStamped()
        p.header.stamp    = self.latest_sim_time_msg  # Используем украденное время
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
            self.recorded_poses.append(self._make_pose_stamped(self.x, self.y, self.theta))
            self.path_msg.poses = self.recorded_poses
            self._path_dirty = True

    def _send_nav_path(self):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            return
        goal = NavigateThroughPoses.Goal()
        goal.poses = self.recorded_poses
        self.state = 'NAVIGATING'
        self.nav_client.send_goal_async(goal).add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
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
        self.current_goal_handle = None
        self.state = 'IDLE'
        self.recorded_poses.clear()
        self.path_msg.poses.clear()
        self._path_dirty = True

    # ------------------------------------------------------------------ #
    #  Главный цикл (вызывается из OS-потока, 30Hz)                      #
    # ------------------------------------------------------------------ #

    def _update(self):
        # === ВОРУЕМ ВРЕМЯ ИЗ TF ===
        try:
            # Time(0) дает нам самый свежий доступный TF (Обычно 30-50 Гц)
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            self.latest_sim_time_msg = t.header.stamp  # Сохраняем штамп времени симуляции!
            self.robot_x = t.transform.translation.x
            self.robot_y = t.transform.translation.y
            self.robot_yaw = self._yaw_from_quat(t.transform.rotation)
            self.robot_pose_valid = True
        except Exception:
            self.robot_pose_valid = False

        # Если мы еще не получили ни одного TF, нам нечем штамповать сообщения
        if self.latest_sim_time_msg is None:
            return

        # Физика всегда считается по реальному времени процессора
        real_ns = time.monotonic_ns()
        if self._last_real_ns is None:
            self._last_real_ns = real_ns
            return
        dt = (real_ns - self._last_real_ns) / 1e9
        self._last_real_ns = real_ns

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

        # --- TF ---
        self._tf_msg.header.stamp          = self.latest_sim_time_msg
        self._tf_msg.transform.translation.x = self.x
        self._tf_msg.transform.translation.y = self.y
        self._tf_msg.transform.rotation.z  = qz
        self._tf_msg.transform.rotation.w  = qw
        self.tf_broadcaster.sendTransform(self._tf_msg)

        # --- Визуализация 10Hz ---
        self._ui_counter += 1
        if self._ui_counter < 3:
            if self._path_dirty and self.recorded_poses:
                self.path_pub.publish(self.path_msg)
                self._path_dirty = False
            return
        self._ui_counter = 0

        is_moving   = abs(self.v) > 0.01 or abs(self.w) > 0.01
        dist_robot  = math.hypot(self.x - self.robot_x, self.y - self.robot_y)
        hide        = self.robot_pose_valid and dist_robot < 0.35 and not is_moving

        m = self._marker
        m.header.stamp    = self.latest_sim_time_msg
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
                        self._make_pose_stamped(self.robot_x, self.robot_y, self.robot_yaw))
                if self.recorded_poses:
                    last = self.recorded_poses[-1].pose.position
                    if math.hypot(self.x - last.x, self.y - last.y) >= self.record_distance:
                        self.recorded_poses.append(
                            self._make_pose_stamped(self.x, self.y, self.theta))
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
    node = ShadowTeleopSim()
    try:
        rclpy.spin(node)
    finally:
        node._running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()