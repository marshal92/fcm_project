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

        # Кэш сессии — только для частых чтений, не для логики stop/start
        self._session_cache = {'running': False, 'ts': 0.0, 'ttl': 2.0}

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.sub = self.create_subscription(String, '/system_command', self.command_cb, qos)

        self.get_logger().info(f"Mission Manager запущен. sim_time={self.use_sim_time}")

    # --- Проверка сессии ---

    def _check_session_real(self):
        """Реальная проверка tmux (без кэша)"""
        r = subprocess.run(
            ['tmux', 'has-session', '-t', self.session_name],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.0
        )
        return r.returncode == 0

    def _check_session_cached(self):
        """Кэшированная проверка для нечастых запросов (статус UI и т.п.)"""
        now = time.monotonic()  # monotonic — не зависит от sim_time
        if now - self._session_cache['ts'] < self._session_cache['ttl']:
            return self._session_cache['running']
        result = self._check_session_real()
        self._session_cache.update({'running': result, 'ts': now})
        return result

    def _invalidate_cache(self):
        self._session_cache['ts'] = 0.0

    # --- Запуск / Остановка ---

    def _start_mission(self, map_name="shelter_map"):
        with self._lock:
            if self._check_session_real():  # при запуске — всегда реальная проверка
                self.get_logger().warn("Миссия УЖЕ запущена!")
                return
            sim = "true" if self.use_sim_time else "false"
            cmd = f"ros2 launch fcm_digital_twin mission_nav.launch.py map_name:={map_name} use_sim_time:={sim}"
            subprocess.Popen(
                ['tmux', 'new-session', '-d', '-s', self.session_name, cmd],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL
            )
            self._invalidate_cache()
            self.get_logger().info(f"🚀 Миссия запущена: {map_name}")
    
    def _start_amcl_mission(self, map_name="shelter_map"):
        """Запуск классического AMCL + Nav2"""
        with self._lock:
            if self._check_session_real():
                self.get_logger().warn("Миссия УЖЕ запущена!")
                return
            sim = "true" if self.use_sim_time else "false"
            cmd = f"ros2 launch fcm_digital_twin mission_amcl.launch.py map_name:={map_name} use_sim_time:={sim}"
            subprocess.Popen(
                ['tmux', 'new-session', '-d', '-s', self.session_name, cmd],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL
            )
            self._invalidate_cache()
            self.get_logger().info(f"🗺️ AMCL Миссия запущена: {map_name}")

    def _stop_mission(self):
        with self._lock:
            if not self._check_session_real():
                self.get_logger().warn("Нет активных миссий.")
                return
            self.get_logger().info("🛑 Остановка (Ctrl+C)...")
            subprocess.run(
                ['tmux', 'send-keys', '-t', self.session_name, 'C-c'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )

        # Ждём завершения В СИСТЕМНОМ ВРЕМЕНИ (не sim_time!) — отдельный поток
        def _wait():
            for _ in range(10):
                time.sleep(1)
                if not self._check_session_real():  # реальная проверка, не кэш
                    self.get_logger().info("✅ Миссия завершена.")
                    self._invalidate_cache()
                    return
            self.get_logger().error("Принудительное уничтожение!")
            subprocess.run(['tmux', 'kill-session', '-t', self.session_name],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self._invalidate_cache()

        threading.Thread(target=_wait, daemon=True).start()

    def _restart_mission(self):
        """Restart в отдельном потоке — sleep не блокирует executor"""
        def _do():
            self._stop_mission()
            time.sleep(3)  # ждём освобождения портов (системное время)
            self._start_mission("shelter_map")
        threading.Thread(target=_do, daemon=True).start()

    # --- Вспомогательные команды ---

    def _set_param(self, node_name, param, value):
        subprocess.Popen(
            ['ros2', 'param', 'set', node_name, param, value],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

    def _call_service(self, service, srv_type):
        subprocess.Popen(
            ['ros2', 'service', 'call', service, srv_type],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )

    def _start_freeride(self):
        with self._lock:
            if self._check_session_real():
                self.get_logger().warn("Миссия уже запущена!")
                return
            sim = "true" if self.use_sim_time else "false"
            cmd = f"ros2 launch fcm_digital_twin mission_freeride.launch.py use_sim_time:={sim}"
            subprocess.Popen(
                ['tmux', 'new-session', '-d', '-s', self.session_name, cmd],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL
            )
            self._invalidate_cache()
            self.get_logger().info("🚀 Freeride запущен")

    # --- Колбек команд ---

    def command_cb(self, msg):
        cmd = msg.data.strip().lower()

        dispatch = {
            'start_shelter':  lambda: threading.Thread(target=self._start_mission, args=("shelter_map",), daemon=True).start(),
            'start_kitchen':  lambda: threading.Thread(target=self._start_mission, args=("kitchen_map",), daemon=True).start(),
            'start_amcl_shelter': lambda: threading.Thread(target=self._start_amcl_mission, args=("shelter_map",), daemon=True).start(),
            'stop':           lambda: threading.Thread(target=self._stop_mission, daemon=True).start(),
            'restart':        lambda: self._restart_mission(),
            'start_freeride': lambda: threading.Thread(target=self._start_freeride, daemon=True).start(),
            'rad_on':         lambda: self._set_param('/radiation_field_server', 'is_active', 'true'),
            'rad_off':        lambda: self._set_param('/radiation_field_server', 'is_active', 'false'),
            'clear_costmaps': lambda: [
                self._call_service('/local_costmap/clear_entirely_local_costmap', 'nav2_msgs/srv/ClearEntireCostmap'),
                self._call_service('/global_costmap/clear_entirely_global_costmap', 'nav2_msgs/srv/ClearEntireCostmap'),
            ],
            'toggle_slam':    lambda: self._call_service('/slam_toolbox/pause_new_measurements', 'slam_toolbox/srv/Pause'),
        }

        if cmd in dispatch:
            dispatch[cmd]()
            self.get_logger().info(f"→ {cmd}")
        else:
            self.get_logger().error(f"Неизвестная команда: '{cmd}'")


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)  # SingleThreaded — нет постоянных таймеров, executor почти не работает
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown...")
        node._stop_mission()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()