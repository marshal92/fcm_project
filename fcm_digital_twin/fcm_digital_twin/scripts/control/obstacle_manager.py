#!/usr/bin/env python3
import subprocess
import sys
import os

# THAT IS THE INTERNAL NAME OF THE WORLD
WORLD_NAME = "shelter_zero" 

X = 11.5
Y = 14.0

SDF_CUBE = """
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="phantom_cube">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>1.0 1.0 1.0</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>1.0 1.0 1.0</size></box></geometry>
        <material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
"""

def spawn():
    print(f"Save the cube SDF to a temporary file...")
    temp_path = '/tmp/phantom_cube.sdf'
    with open(temp_path, 'w') as f:
        f.write(SDF_CUBE)
        
    print(f"Spawning the cube at coordinates X:{X}, Y:{Y}...")
    # We use the native EntityFactory service from Gazebo
    req = f'sdf_filename: "{temp_path}" name: "phantom_cube" pose: {{position: {{x: {X}, y: {Y}, z: 0.5}}}}'
    cmd = [
        'gz', 'service', '-s', f'/world/{WORLD_NAME}/create',
        '--reqtype', 'gz.msgs.EntityFactory',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000',
        '--req', req
    ]
    subprocess.run(cmd)
    print("The cube has successfully appeared in the simulation.")

def remove():
    print(" We remove the cube....")
    req = 'name: "phantom_cube" type: MODEL'
    cmd = [
        'gz', 'service', '-s', f'/world/{WORLD_NAME}/remove',
        '--reqtype', 'gz.msgs.Entity',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000',
        '--req', req
    ]
    subprocess.run(cmd)
    print("Cube removed from simulation!")

def main(args=None):
    if args is None:
        args = sys.argv
        
    if len(args) < 2:
        print("❌ Error! Usage: ros2 run fcm_digital_twin obstacle_manager [spawn | remove]")
        return

    command = args[-1]

    if command == 'spawn':
        spawn()
    elif command == 'remove':
        remove()
    else:
        print(f"❌ Error! Unknown command: {command}. Use 'spawn' or 'remove'.")

if __name__ == '__main__':
    main()