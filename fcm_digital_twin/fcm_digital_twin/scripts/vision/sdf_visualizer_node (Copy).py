import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy
import xml.etree.ElementTree as ET
import math
import os

class SdfVisualizerNode(Node):
    def __init__(self):
        super().__init__('sdf_visualizer_node')
        
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, '/digital_twin/environment_3d', qos_profile)
        
        self.sdf_file_path = '/home/oleksandr/ros2_ws/src/fcm_project/fcm_digital_twin/worlds/shelter.sdf'
        
        # ВЕРНУЛИ ТАЙМЕР: Публикуем каждые 2 секунды. Теперь галочка в RViz работает безотказно!
        self.timer = self.create_timer(2.0, self.publish_world)
        self.get_logger().info('SDF Smart Parser Started (Publishing every 2s)...')

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def publish_world(self):
        if not os.path.exists(self.sdf_file_path):
            self.get_logger().error(f'SDF file not found: {self.sdf_file_path}')
            return

        marker_array = MarkerArray()
        tree = ET.parse(self.sdf_file_path)
        root = tree.getroot()

        marker_id = 0
        
        for model in root.findall('.//model'):
            model_name = model.attrib.get('name', f'unknown_model_{marker_id}')
            if model_name == 'ground_plane':
                continue

            model_pose_tag = model.find('pose')
            model_pose = model_pose_tag.text if model_pose_tag is not None else "0 0 0 0 0 0"
            pose_vals = [float(v) for v in model_pose.split()]
            
            for visual in model.findall('.//visual'):
                mesh = visual.find('.//mesh')
                if mesh is None:
                    continue
                    
                uri_tag = mesh.find('uri')
                if uri_tag is None:
                    continue
                
                mesh_uri = uri_tag.text
                
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                
                marker.ns = model_name 
                marker.id = marker_id
                marker.type = Marker.MESH_RESOURCE
                marker.action = Marker.ADD
                marker.mesh_resource = mesh_uri
                
                if mesh_uri.lower().endswith('.dae'):
                    marker.mesh_use_embedded_materials = True
                else:
                    marker.mesh_use_embedded_materials = False
                
                marker.pose.position.x = pose_vals[0]
                marker.pose.position.y = pose_vals[1]
                marker.pose.position.z = pose_vals[2] + 0.01 
                
                q = self.euler_to_quaternion(pose_vals[3], pose_vals[4], pose_vals[5])
                marker.pose.orientation.x = q[0]
                marker.pose.orientation.y = q[1]
                marker.pose.orientation.z = q[2]
                marker.pose.orientation.w = q[3]
                
                scale_tag = mesh.find('scale')
                if scale_tag is not None:
                    scale_vals = [float(v) for v in scale_tag.text.split()]
                    marker.scale.x = scale_vals[0]
                    marker.scale.y = scale_vals[1]
                    marker.scale.z = scale_vals[2]
                else:
                    marker.scale.x = 1.0
                    marker.scale.y = 1.0
                    marker.scale.z = 1.0

                # ИСПРАВЛЕНИЕ: Абсолютно непрозрачный цвет
                marker.color.r = 0.50
                marker.color.g = 0.50
                marker.color.b = 0.50
                marker.color.a = 1.0 # 1.0 = Непрозрачно. Убираем баг наложения слоев RViz

                marker_array.markers.append(marker)
                marker_id += 1

        if marker_id > 0:
            self.publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = SdfVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()