#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import math

def euler_from_quaternion(x, y, z, w):
    """Конвертация кватерниона в углы Эйлера (Roll, Pitch, Yaw)"""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z # Нам нужен только рысканье (Yaw)

def quaternion_from_euler(roll, pitch, yaw):
    """Конвертация углов Эйлера обратно в кватернион"""
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class StabilizedFramePublisher(Node):
    def __init__(self):
        super().__init__('stabilized_frame_publisher')
        
        # Инструменты для чтения и записи дерева TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Таймер: 50 Гц (0.02 секунды)
        self.timer = self.create_timer(0.02, self.publish_stabilized_frame)

    def publish_stabilized_frame(self):
        try:
            # 1. Узнаем, где сейчас находится танк (base_footprint) относительно мира (odom)
            t = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())

            # 2. Вытаскиваем его реальный поворот (Yaw)
            q = t.transform.rotation
            yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

            # 3. Создаем кватернион, где Roll = 0, Pitch = 0, а Yaw = реальный
            new_q = quaternion_from_euler(0.0, 0.0, yaw)

            # 4. Формируем новый идеальный фрейм
            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'             # Родитель - мир
            msg.child_frame_id = 'base_stabilized'   # Ребенок - наш новый фрейм

            # Координаты X,Y,Z точно такие же, как у танка
            msg.transform.translation.x = t.transform.translation.x
            msg.transform.translation.y = t.transform.translation.y
            msg.transform.translation.z = t.transform.translation.z

            # А вот вращение - выровненное!
            msg.transform.rotation.x = new_q[0]
            msg.transform.rotation.y = new_q[1]
            msg.transform.rotation.z = new_q[2]
            msg.transform.rotation.w = new_q[3]

            # 5. Публикуем в систему
            self.tf_broadcaster.sendTransform(msg)

        except Exception as e:
            # Ошибки трансформации игнорируем (бывают в первые секунды запуска)
            pass

def main(args=None):
    rclpy.init(args=args)
    node = StabilizedFramePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()