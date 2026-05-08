import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy
import xml.etree.ElementTree as ET
import math
import os
from ament_index_python.packages import get_package_share_directory

class SdfVisualizerNode(Node):
    def __init__(self):
        super().__init__('sdf_visualizer_node')
        
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, '/digital_twin/environment_3d', qos_profile)
        
        self.declare_parameter('world_file', '213.sdf')
        
        self.declare_parameter('x_offset', 0.0)
        self.declare_parameter('y_offset', 0.0)
        self.declare_parameter('z_offset', 0.0)

        world_filename = self.get_parameter('world_file').get_parameter_value().string_value
        pkg_share_dir = get_package_share_directory('fcm_digital_twin')
        self.sdf_file_path = os.path.join(pkg_share_dir, 'worlds', world_filename)
        
        self.timer = self.create_timer(2.0, self.publish_world)
        self.get_logger().info(f'SDF Smart Parser Started. Target world: {self.sdf_file_path}')

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

        x_off = self.get_parameter('x_offset').value
        y_off = self.get_parameter('y_offset').value
        z_off = self.get_parameter('z_offset').value

        marker_array = MarkerArray()
        tree = ET.parse(self.sdf_file_path)
        root = tree.getroot()

        marker_id = 0
        
        for model in root.findall('.//model'):
            model_name = model.attrib.get('name', f'unknown_model_{marker_id}')
            if model_name == 'ground_plane':
                continue

            model_pose_tag = model.find('pose')
            m_pose = [float(v) for v in (model_pose_tag.text if model_pose_tag is not None else "0 0 0 0 0 0").split()]
            
            for link in model.findall('.//link'):
                link_pose_tag = link.find('pose')
                l_pose = [float(v) for v in (link_pose_tag.text if link_pose_tag is not None else "0 0 0 0 0 0").split()]

                for visual in link.findall('.//visual'):
                    visual_name = visual.attrib.get('name', f'visual_{marker_id}')
                    
                    v_pose_tag = visual.find('pose')
                    v_pose = [float(v) for v in (v_pose_tag.text if v_pose_tag is not None else "0 0 0 0 0 0").split()]

                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    
                    marker.ns = f"{model_name}/{visual_name}"
                    marker.id = marker_id
                    marker.action = Marker.ADD

                    mesh = visual.find('.//mesh')
                    box = visual.find('.//box')

                    if mesh is not None:
                        uri_tag = mesh.find('uri')
                        if uri_tag is None: continue
                        
                        marker.type = Marker.MESH_RESOURCE
                        mesh_uri = uri_tag.text.replace('model://', 'package://')
                        marker.mesh_resource = mesh_uri
                        marker.mesh_use_embedded_materials = mesh_uri.lower().endswith('.dae')
                        
                        scale_tag = mesh.find('scale')
                        if scale_tag is not None:
                            sx, sy, sz = [float(v) for v in scale_tag.text.split()]
                            marker.scale.x = sx; marker.scale.y = sy; marker.scale.z = sz
                        else:
                            marker.scale.x = 1.0; marker.scale.y = 1.0; marker.scale.z = 1.0

                    elif box is not None:
                        marker.type = Marker.CUBE
                        size_tag = box.find('size')
                        if size_tag is not None:
                            sx, sy, sz = [float(v) for v in size_tag.text.split()]
                            marker.scale.x = sx; marker.scale.y = sy; marker.scale.z = sz
                        else:
                            marker.scale.x = 1.0; marker.scale.y = 1.0; marker.scale.z = 1.0
                    else:
                        continue 

                    marker.pose.position.x = m_pose[0] + l_pose[0] + v_pose[0] + x_off
                    marker.pose.position.y = m_pose[1] + l_pose[1] + v_pose[1] + y_off
                    marker.pose.position.z = m_pose[2] + l_pose[2] + v_pose[2] + z_off
                    
                    q = self.euler_to_quaternion(m_pose[3], m_pose[4], m_pose[5])
                    marker.pose.orientation.x = q[0]
                    marker.pose.orientation.y = q[1]
                    marker.pose.orientation.z = q[2]
                    marker.pose.orientation.w = q[3]

                    # Color
                    material = visual.find('.//material')
                    if material is not None:
                        color_tag = material.find('diffuse')
                        if color_tag is None:
                            color_tag = material.find('ambient')
                        
                        if color_tag is not None and color_tag.text:
                            rgba = [float(v) for v in color_tag.text.split()]
                            marker.color.r = rgba[0]
                            marker.color.g = rgba[1]
                            marker.color.b = rgba[2]
                            marker.color.a = rgba[3] if len(rgba) > 3 else 1.0
                        else:
                            marker.color.r = 0.5; marker.color.g = 0.5; marker.color.b = 0.5; marker.color.a = 1.0
                    else:
                        marker.color.r = 0.5; marker.color.g = 0.5; marker.color.b = 0.5; marker.color.a = 1.0
                    
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