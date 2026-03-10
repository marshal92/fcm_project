#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import time

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        
        # Безопасное чтение параметра (проверяем, не объявил ли его ROS 2 до нас)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        
        # Получаем сырое значение и жестко парсим строку в boolean
        param_val = self.get_parameter('use_sim_time').value
        self.use_sim_time = str(param_val).lower() == 'true'

       
        # Подписка на команды из Foxglove
        self.sub = self.create_subscription(String, '/system_command', self.command_cb, 10)
        
        self.session_name = "mission_nav_session"
        
        self.get_logger().info(f"Mission Manager запущен. Симуляция: {self.use_sim_time}")
        self.get_logger().info("Ожидание команд в топике /system_command...")

    def is_session_running(self):
        """Проверяет, существует ли сессия tmux с миссией"""
        result = subprocess.run(['tmux', 'has-session', '-t', self.session_name], stderr=subprocess.PIPE, stdout=subprocess.PIPE)
        return result.returncode == 0

    def start_mission(self, map_name="shelter_map"):
        if self.is_session_running():
            self.get_logger().warn("Миссия УЖЕ запущена! Сначала остановите текущую.")
            return

        sim_time_str = "true" if self.use_sim_time else "false"
        
        self.get_logger().info(f"🚀 Запуск миссии: Карта '{map_name}', use_sim_time: {sim_time_str}")
        
        # Формируем команду. Запускаем лаунч и после его завершения сессия закроется сама.
        cmd = f"ros2 launch fcm_digital_twin mission_nav.launch.py map_name:={map_name} use_sim_time:={sim_time_str}"
        
        # Запускаем в отсоединенной (detached) сессии tmux
        subprocess.run(['tmux', 'new-session', '-d', '-s', self.session_name, cmd])
        self.get_logger().info(f"Сессия '{self.session_name}' создана в фоне.")

    def stop_mission(self):
        if not self.is_session_running():
            self.get_logger().warn("Нет активных миссий для остановки.")
            return

        self.get_logger().info("🛑 Остановка миссии (отправка Ctrl+C для Graceful Shutdown)...")
        
        # Отправляем Ctrl+C (SIGINT) в tmux. Это запустит правильный процесс выключения Lifecycle узлов!
        subprocess.run(['tmux', 'send-keys', '-t', self.session_name, 'C-c'])
        
        # Ждем, пока ROS 2 аккуратно выключится (обычно 3-5 секунд)
        self.get_logger().info("Ожидание завершения узлов...")
        for i in range(10):
            time.sleep(1)
            if not self.is_session_running():
                self.get_logger().info("Миссия успешно и безопасно завершена.")
                return
                
        # Если через 10 секунд сессия все еще висит (кто-то завис намертво) - убиваем жестко
        self.get_logger().error("Миссия не завершилась сама! Принудительное уничтожение (kill).")
        subprocess.run(['tmux', 'kill-session', '-t', self.session_name])

    def command_cb(self, msg):
        cmd = msg.data.lower()
        
        if cmd == 'start_shelter':
            self.start_mission(map_name="shelter_map")
        elif cmd == 'stop':
            self.stop_mission()
        elif cmd == 'restart':
            self.stop_mission()
            time.sleep(2) # Даем портам освободиться
            self.start_mission(map_name="shelter_map")

        # === ДОБАВЛЯЕМ КОМАНДЫ РАДИАЦИИ ===
        elif cmd == 'rad_on':
            self.get_logger().info("☢️ Радиация ВКЛЮЧЕНА")
            subprocess.Popen(['ros2', 'param', 'set', '/radiation_field_server', 'is_active', 'true'])
        elif cmd == 'rad_off':
            self.get_logger().info("🟢 Радиация ВЫКЛЮЧЕНА")
            subprocess.Popen(['ros2', 'param', 'set', '/radiation_field_server', 'is_active', 'false'])
        
        # === ОЧИСТКА COSTMAPS ===
        elif cmd == 'clear_costmaps':
            self.get_logger().info("🧹 Очистка призрачных препятствий (Costmaps)...")
            # Используем Popen, чтобы не блокировать менеджер, если сервис чуть задержится
            subprocess.Popen(['ros2', 'service', 'call', '/local_costmap/clear_entirely_local_costmap', 'nav2_msgs/srv/ClearEntireCostmap'])
            subprocess.Popen(['ros2', 'service', 'call', '/global_costmap/clear_entirely_global_costmap', 'nav2_msgs/srv/ClearEntireCostmap'])

        # === ЗАПУСК FREERIDE ===
        elif cmd == 'start_freeride':
            if self.is_session_running():
                self.get_logger().warn("Миссия УЖЕ запущена! Сначала остановите текущую.")
                return
            sim_time_str = "true" if self.use_sim_time else "false"
            self.get_logger().info("🚀 Запуск миссии FREERIDE (Свободная езда с нуля)...")
            launch_cmd = f"ros2 launch fcm_digital_twin mission_freeride.launch.py use_sim_time:={sim_time_str}"
            subprocess.run(['tmux', 'new-session', '-d', '-s', self.session_name, launch_cmd])
        
        # === УПРАВЛЕНИЕ SLAM ===
        elif cmd == 'toggle_slam':
            self.get_logger().info("⏸️/▶️ Переключение паузы сканирования SLAM...")
            subprocess.Popen(['ros2', 'service', 'call', '/slam_toolbox/pause_new_measurements', 'slam_toolbox/srv/Pause'])    

        else:
            self.get_logger().error(f"Неизвестная команда: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Остановка Mission Manager...")
        node.stop_mission() # Убиваем миссию, если убивают сам менеджер
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()