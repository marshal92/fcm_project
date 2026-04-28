#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import time
import json
import threading
import math

class HeartbeatMonitor(Node):
    def __init__(self):
        super().__init__('mission_control')
        self.last_heartbeat = self.get_clock().now().nanoseconds / 1e9
        self.is_connected = False
        self.last_rviz_goal = None  
        self.deployed_tools = set()
        
        self.pending_task = None
        self.status = 'NORMAL' # NORMAL, WARNING, EVACUATING
        
        self.create_subscription(Empty, '/operator_heartbeat', self.heartbeat_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(String, '/tactical_command', self.tactical_cb, 10)

    def heartbeat_cb(self, msg):
        self.last_heartbeat = self.get_clock().now().nanoseconds / 1e9
        if not self.is_connected:
            self.get_logger().info('\033[1;32m[Connection] Heartbeat received. Control is active.\033[0m')
            self.is_connected = True

    def goal_cb(self, msg):
        self.last_rviz_goal = msg
        self.pending_task = {'type': 'go_to', 'pose': msg, 'bt': ''}
        self.get_logger().info('New route goal saved.')

    def tactical_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            action = payload.get("action")

            if action in ["go_to", "go_to_with_bt"]:
                x = float(payload.get("x", 0.0))
                y = float(payload.get("y", 0.0))
                yaw = float(payload.get("yaw", 0.0))
                bt_xml = payload.get("bt", "")
                
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.pose.position.x = x
                goal_pose.pose.position.y = y

                goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
                goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
            
                self.pending_task = {'type': 'go_to', 'pose': goal_pose, 'bt': bt_xml}
                self.get_logger().info(f"📍 Transit: X={x}, Y={y}, Angle={yaw} rad")

            elif action == "waypoints":
                pts = payload.get("points", [])
                poses = []
                for p in pts:
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.pose.position.x = float(p[0])
                    pose.pose.position.y = float(p[1])
                    yaw = float(p[2]) if len(p) > 2 else 0.0
                    pose.pose.orientation.z = math.sin(yaw / 2.0)
                    pose.pose.orientation.w = math.cos(yaw / 2.0)
                    poses.append(pose)
                
                self.pending_task = {'type': 'waypoints', 'poses': poses}
                self.get_logger().info(f"Route from {len(poses)} points added to queue.")

            elif action == "mock_sample":
                self.get_logger().info("🛠 Command: LFCM Sample Collection")
                threading.Thread(target=self._execute_mock_sample, daemon=True).start()

            elif action == "cancel":
                self.pending_task = {'type': 'cancel'}
                self.get_logger().info("🛑 Received command to cancel navigation.")

            else:
                self.get_logger().warn(f"Unknown command: {action}")

        except Exception as e:
            self.get_logger().error(f"Error parsing message: {e}")

    def _execute_mock_sample(self):
        tool_name = "lfcm_drill"
        self.deployed_tools.add(tool_name) 
        try:
            self.get_logger().info("[Payload_Node]: Initializing sampling procedure...")
            time.sleep(2.0)
            self.get_logger().info("[Payload_Node]: Calibrating drilling angle...")
            time.sleep(3.0)
            self.get_logger().info("[Payload_Node]: Sampling complete. Container isolated.")
            self.get_logger().info("✅ Status: Payload Action Complete.")
        finally:
            self.deployed_tools.remove(tool_name) 

def main(args=None):
    rclpy.init(args=args)
    
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active(localizer='bt_navigator')
    
    monitor = HeartbeatMonitor()
    
    # 2 stage protocol timeouts
    warning_timeout = 2.0  
    evac_timeout = 10.0     
    
    bt_survival = ''

    monitor.get_logger().info('Mission Control started.')

    while rclpy.ok():
        try:
            rclpy.spin_once(monitor, timeout_sec=0.5)
            
            # --- 1. Tactical command processing ---
            if monitor.pending_task is not None and monitor.status == 'NORMAL' and monitor.is_connected:
                task = monitor.pending_task
                monitor.pending_task = None
                               
                #If received CANCEL command — simply stop and clear memory
                if task['type'] == 'cancel':
                    navigator.cancelTask()
                    monitor.last_rviz_goal = None
                    monitor.get_logger().info('🛑 Cancelled navigation by user.')
                    continue

                # Interruption Before any NEW task, the old one is clearly obliterated,
                if not navigator.isTaskComplete():
                    navigator.cancelTask()
                    time.sleep(0.1)
                
                #Performing a new task
                if task['type'] == 'go_to':
                    task['pose'].header.stamp = navigator.get_clock().now().to_msg()
                    monitor.last_rviz_goal = task['pose']
                    navigator.goToPose(task['pose'], behavior_tree=task['bt'])
                
                elif task['type'] == 'waypoints':
                    for p in task['poses']:
                        p.header.stamp = navigator.get_clock().now().to_msg()
                    # We use goThroughPoses for smooth movement without stopping
                    navigator.goThroughPoses(task['poses'])
                    monitor.last_rviz_goal = task['poses'][-1]        
                                   
            # --- 2. MONITORING CONNECTION ---
            current_time = monitor.get_clock().now().nanoseconds / 1e9
            elapsed = current_time - monitor.last_heartbeat
            
            # STAGE 2: CRITICAL BREAK (> 10 seconds)
            if elapsed > evac_timeout and monitor.status != 'EVACUATING':
                if len(monitor.deployed_tools) > 0:
                    continue 

                monitor.is_connected = False
                monitor.status = 'EVACUATING'
                monitor.get_logger().warn('\033[1;31m[Connection] Critical Connection Loss! Evacuation to base...\033[0m')
                
                navigator.cancelTask()
                time.sleep(0.5)
                
                home_pose = PoseStamped()
                home_pose.header.frame_id = 'map'
                home_pose.pose.position.x = 0.0 
                home_pose.pose.position.y = 0.0
                home_pose.pose.orientation.w = 1.0
                home_pose.header.stamp = navigator.get_clock().now().to_msg()
                navigator.goToPose(home_pose, behavior_tree=bt_survival)

            # STAGE 1: SIGNAL LOSS AND WAITING (2 to 10 seconds)
            elif elapsed > warning_timeout and elapsed <= evac_timeout and monitor.status == 'NORMAL':
                monitor.status = 'WARNING'
                monitor.get_logger().warn('\033[1;33m[Connection] SIGNAL LOSS! Emergency stop. Waiting (10 sec)...\033[0m')
                navigator.cancelTask() #instantly apply brakes!

            # SIGNAL RESTORED (< 2 seconds)
            elif elapsed <= warning_timeout and monitor.status != 'NORMAL':
                monitor.is_connected = True
                
                if monitor.status == 'EVACUATING':
                    monitor.get_logger().info('\033[1;34m[Connection] SIGNAL RESTORED! Cancelling evacuation.\033[0m')
                    navigator.cancelTask()
                    time.sleep(0.5)
                elif monitor.status == 'WARNING':
                    monitor.get_logger().info('\033[1;32m[Connection] SIGNAL RESTORED! Removing blockage.\033[0m')
                
                monitor.status = 'NORMAL'
                
                if monitor.last_rviz_goal:
                    monitor.get_logger().info('Recovering route from memory...')
                    monitor.last_rviz_goal.header.stamp = navigator.get_clock().now().to_msg()
                    navigator.goToPose(monitor.last_rviz_goal, behavior_tree='') 
                else:
                    monitor.get_logger().info('Waiting for new instructions.')

        except RuntimeError as e:
            monitor.get_logger().error(f'\033[1;41m[DDS ERROR] Network/ROS failure: {e}. Node surviving...\033[0m')
            time.sleep(1.0) 
        except Exception as e:
            monitor.get_logger().error(f'\033[1;41m[FATAL ERROR] Unexpected exception: {e}. Node surviving...\033[0m')
            time.sleep(1.0)

    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()