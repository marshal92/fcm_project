#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import math

def euler_from_quaternion(x, y, z, w):
    """Conversion of quaternion to Euler angles (Roll, Pitch, Yaw)"""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

def quaternion_from_euler(roll, pitch, yaw):
    """Conversion of Euler angles to quaternion"""
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return [qx, qy, qz, qw]

class StabilizedFramePublisher(Node):
    def __init__(self):
        super().__init__('stabilized_frame_publisher')
        
        # Tools for reading and writing the TF tree
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer: 50 Hz (0.02 seconds)
        self.timer = self.create_timer(0.02, self.publish_stabilized_frame)

    def publish_stabilized_frame(self):
        try:
            # 1. Find out where the tank is currently located (base_footprint) relative to the world (odom)
            t = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())

            # 2. Extract its real rotation (Yaw)
            q = t.transform.rotation
            yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

            # 3. Create a quaternion where Roll = 0, Pitch = 0, and Yaw = real
            new_q = quaternion_from_euler(0.0, 0.0, yaw)

            # 4. Form a new ideal frame
            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'             
            msg.child_frame_id = 'base_stabilized'   

            # Coordinates X,Y,Z exactly the same as the tank
            msg.transform.translation.x = t.transform.translation.x
            msg.transform.translation.y = t.transform.translation.y
            msg.transform.translation.z = t.transform.translation.z

            # But the rotation is aligned!
            msg.transform.rotation.x = new_q[0]
            msg.transform.rotation.y = new_q[1]
            msg.transform.rotation.z = new_q[2]
            msg.transform.rotation.w = new_q[3]

            # 5. Publish to the system
            self.tf_broadcaster.sendTransform(msg)

        except Exception as e:
            # Ignore transformation errors (can happen in the first few seconds of startup)
            pass

def main(args=None):
    rclpy.init(args=args)
    node = StabilizedFramePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()