#!/bin/bash

export ROS_DOMAIN_ID=42
ROBOT_IP="192.168.1.100"

echo "Starting Real UR12e + Robotiq System..."

# Terminal 1: Robot system
gnome-terminal -- bash -c "
export ROS_DOMAIN_ID=42
source /ros2_ws/install/setup.bash
ros2 launch my_ur_description my_robot.launch.py \
  use_fake_hardware:=false \
  robot_ip:=$ROBOT_IP
exec bash"

# Wait for robot to start
sleep 10

# Terminal 2: Gripper server
gnome-terminal -- bash -c "
export ROS_DOMAIN_ID=42
source /ros2_ws/install/setup.bash
ros2 run my_ur_description robotiq_real_gripper_server.py --ros-args \
  -p robot_ip:=$ROBOT_IP
exec bash"

echo "System launched!"
echo "Press Play on robot teach pendant"