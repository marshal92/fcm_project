#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import subprocess
import time
import threading

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        if not self.has_parameter('is_simulation'):
            self.declare_parameter('is_simulation', False)
        param_val = self.get_parameter('is_simulation').value
        self.use_sim_time = str(param_val).lower() == 'true'

        self._lock = threading.Lock()
        self.session_name = "mission_nav_session"

        # Session cache - only for frequent reads, not for stop/start logic
        self._session_cache = {'running': False, 'ts': 0.0, 'ttl': 2.0}

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.sub = self.create_subscription(String, '/system_command', self.command_cb, qos)

        self.get_logger().info(f"Mission Manager started. sim_time={self.use_sim_time}")

    # Session checking with caching for UI status, but real check for start/stop logic

    def _check_session_real(self):
        """Real-time check of tmux session (no cache)"""
        r = subprocess.run(
            ['tmux', 'has-session', '-t', self.session_name],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.0
        )
        return r.returncode == 0

    def _check_session_cached(self):
        """Cached check for infrequent queries (UI status, etc.)"""
        now = time.monotonic()  # monotonic — not dependent on sim_time
        if now - self._session_cache['ts'] < self._session_cache['ttl']:
            return self._session_cache['running']
        result = self._check_session_real()
        self._session_cache.update({'running': result, 'ts': now})
        return result

    def _invalidate_cache(self):
        self._session_cache['ts'] = 0.0

    # Start / Stop missions

    def _start_mission(self, map_name="shelter_map"):
        with self._lock:
            if self._check_session_real():
                self.get_logger().warn("Mission already running!")
                return
            sim = "true" if self.use_sim_time else "false"
            cmd = f"ros2 launch fcm_digital_twin mission_nav.launch.py map_name:={map_name} use_sim_time:={sim}"
            subprocess.Popen(
                ['tmux', 'new-session', '-d', '-s', self.session_name, cmd],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL
            )
            self._invalidate_cache()
            self.get_logger().info(f"🚀 Mission started: {map_name}")
    
    def _start_amcl_mission(self, map_name="shelter_map"):
        """Запуск классического AMCL + Nav2"""
        with self._lock:
            if self._check_session_real():
                self.get_logger().warn("Mission already running!")
                return
            sim = "true" if self.use_sim_time else "false"
            cmd = f"ros2 launch fcm_digital_twin mission_amcl.launch.py map_name:={map_name} use_sim_time:={sim}"
            subprocess.Popen(
                ['tmux', 'new-session', '-d', '-s', self.session_name, cmd],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL
            )
            self._invalidate_cache()
            self.get_logger().info(f"🗺️ AMCL Mission started: {map_name}")

    def _stop_mission(self):
        with self._lock:
            if not self._check_session_real():
                self.get_logger().warn("No active missions.")
                return
            self.get_logger().info("🛑 Stopping mission (Ctrl+C)...")
            subprocess.run(
                ['tmux', 'send-keys', '-t', self.session_name, 'C-c'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )

        # Wait for completion IN SYSTEM TIME (not sim_time!) - separate thread
        def _wait():
            for _ in range(10):
                time.sleep(1)
                if not self._check_session_real():
                    self.get_logger().info("✅ Mission completed.")
                    self._invalidate_cache()
                    return
            self.get_logger().error("Forceful destruction!")
            subprocess.run(['tmux', 'kill-session', '-t', self.session_name],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self._invalidate_cache()

        threading.Thread(target=_wait, daemon=True).start()

    def _restart_mission(self):
        """Restart at separate thread — sleep does not block executor"""
        def _do():
            self._stop_mission()
            time.sleep(3)  
            self._start_mission("shelter_map")
        threading.Thread(target=_do, daemon=True).start()

    # Auxiliary commands

    # FIX: Replace Popen with run to avoid spawning zombie processes
    def _set_param(self, node_name, param, value):
        subprocess.run(
            ['ros2', 'param', 'set', node_name, param, value],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

    # FIX: Replace Popen with run
    def _call_service(self, service, srv_type):
        subprocess.run(
            ['ros2', 'service', 'call', service, srv_type],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

    def _start_freeride(self):
        with self._lock:
            if self._check_session_real():
                self.get_logger().warn("Mission already running!")
                return
            sim = "true" if self.use_sim_time else "false"
            cmd = f"ros2 launch fcm_digital_twin mission_freeride.launch.py use_sim_time:={sim}"
            subprocess.Popen(
                ['tmux', 'new-session', '-d', '-s', self.session_name, cmd],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL
            )
            self._invalidate_cache()
            self.get_logger().info("🚀 Freeride started")

    # Command callback

    def command_cb(self, msg):
        cmd = msg.data.strip().lower()

        dispatch = {
            'start_213': lambda: threading.Thread(target=self._start_mission, args=("213_map",), daemon=True).start(),
            'start_shelter_zero': lambda: threading.Thread(target=self._start_mission, args=("shelter_zero",), daemon=True).start(),
            'start_shelter':  lambda: threading.Thread(target=self._start_mission, args=("shelter_map",), daemon=True).start(),
            'start_kitchen':  lambda: threading.Thread(target=self._start_mission, args=("kitchen_map",), daemon=True).start(),
            'start_amcl_shelter': lambda: threading.Thread(target=self._start_amcl_mission, args=("shelter_map",), daemon=True).start(),
            'stop':           lambda: threading.Thread(target=self._stop_mission, daemon=True).start(),
            'restart':        lambda: self._restart_mission(),
            'start_freeride': lambda: threading.Thread(target=self._start_freeride, daemon=True).start(),
            'rad_on':         lambda: threading.Thread(target=self._set_param, args=('/radiation_field_server', 'is_active', 'true'), daemon=True).start(),
            'rad_off':        lambda: threading.Thread(target=self._set_param, args=('/radiation_field_server', 'is_active', 'false'), daemon=True).start(),
            'clear_costmaps': lambda: [
                threading.Thread(target=self._call_service, args=('/local_costmap/clear_entirely_local_costmap', 'nav2_msgs/srv/ClearEntireCostmap'), daemon=True).start(),
                threading.Thread(target=self._call_service, args=('/global_costmap/clear_entirely_global_costmap', 'nav2_msgs/srv/ClearEntireCostmap'), daemon=True).start(),
            ],
            'toggle_slam':    lambda: threading.Thread(target=self._call_service, args=('/slam_toolbox/pause_new_measurements', 'slam_toolbox/srv/Pause'), daemon=True).start(),
        }

        if cmd in dispatch:
            dispatch[cmd]()
            self.get_logger().info(f"→ {cmd}")
        else:
            self.get_logger().error(f"Unknown command: '{cmd}'")


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown...")
        node._stop_mission()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()