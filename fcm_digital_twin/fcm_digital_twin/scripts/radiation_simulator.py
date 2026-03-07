import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32

class DigitalTwinRadiationNode(Node):
    def __init__(self):
        super().__init__('dt_radiation_node')
        # Теперь мы публикуем не просто цифру, а координаты источника (x, y) и силу (z)
        self.publisher_ = self.create_publisher(Point32, '/radiation_sensor', 10)
        self.timer = self.create_timer(1.0, self.timer_callback) # Публикуем раз в секунду
        
        # Заранее известные координаты ЛПВМ из базы данных Цифрового Двойника
        self.source_x = 1.0
        self.source_y = 0.0
        self.intensity = 500.0 
        self.get_logger().info("Digital Twin Map active: Projecting radiation source to map...")

    def timer_callback(self):
        msg = Point32()
        msg.x = float(self.source_x)
        msg.y = float(self.source_y)
        msg.z = float(self.intensity) # Используем Z для передачи мощности
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DigitalTwinRadiationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()