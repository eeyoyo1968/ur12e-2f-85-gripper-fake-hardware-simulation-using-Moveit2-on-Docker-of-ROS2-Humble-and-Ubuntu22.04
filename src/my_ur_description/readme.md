UR12e Visual-Guarded Sorting System (Jaw-Axis Aligned)
Date: 2026-03-02

This repository contains the integrated control system for a UR12e robot equipped with a Robotiq 2F-85 gripper. The system uses YOLO-based perception to perform jaw-axis aligned sorting into "soft" and "hard" bins.

🏗️ System Architecture
The system relies on six concurrent terminal processes to handle perception, motion planning, hardware bridging, and high-level decision making.

🚀 Setup and Operation
1. Perception Layer (GraspNode4)
Terminal 1: Starts the YOLO bridge that translates vision data into ROS 2 topics.

Bash
export ROS_DOMAIN_ID=43
source /opt/ros/humble/setup.bash
source ~/humble_ws/install/setup.bash
ros2 run grasp_perception grasp_node4
Terminal 2: Verify vision data and jaw-axis orientation.

Bash
export ROS_DOMAIN_ID=43
source /opt/ros/humble/setup.bash
source ~/humble_ws/install/setup.bash
ros2 topic echo /grasp/pose
ros2 topic echo /grasp/jaw_axis
2. Robot Control (MoveIt2 & UR Controller)
Note: Ensure the UR12e is powered on, unlocked, and the "External Control" program is active on the Teach Pendant. The program must include:

set_tool_voltage(24)

set_tool_communication(True, 115200, 0, 1, 1.5, 3.5)

Terminal 3: Launch the UR driver and MoveIt2.

Bash
cd ~/ur12e_ws_onsite2/ur12e-2f-85-gripper-fake-hardware-simulation-using-Moveit2-on-Docker-of-ROS2-Humble-and-Ubuntu22.04
colcon build --symlink-install
source install/setup.bash
ros2 launch my_ur_description my_robot.launch.py use_fake_hardware:=false ur_type:=ur12e robot_ip:=192.168.2.2 reverse_ip:=192.168.2.1 kinematics_params_file:=/ros2_ws/src/my_ur_description/config/calibration/my_ur12e_calibration.yaml
Check for: [spawner_scaled_joint_trajectory_controller]: Configured and activated.

3. Gripper Hardware Bridge (Modbus RTU over TCP)
Terminal 5: Create the virtual serial port /tmp/ttyUR.

Bash
while true; do
  echo "Reviving Bridge..."
  sudo killall -9 socat
  socat pty,link=/tmp/ttyUR,raw,echo=0,waitslave tcp:192.168.2.2:54321,nodelay
  sleep 1
done
Terminal 6: Set permissions and test communication.

Bash
sudo chmod 666 /tmp/ttyUR
python3 -c "import minimalmodbus; i=minimalmodbus.Instrument('/tmp/ttyUR', 9); i.serial.baudrate=115200; i.serial.timeout=1.0; print('Gripper Status:', hex(i.read_register(2000)))"
4. Autonomous Sorting Execution
Terminal 4: Launch the high-level brain. This script calculates jaw-axis alignment and executes the pick-and-place cycle.

Single Run:

Bash
cd ~/ur12e_ws_onsite2/.../src/my_ur_description/scripts
python3 coordinator_brain11_jaw_axis_aligned.py
Continuous Loop (Production):

Bash
./run_picker.sh
🛠️ Key Files Summary
File	Purpose	Source
grasp_node4.py	Perception bridge with non-blocking JSON pipe and jaw-axis output.	
coordinator_brain11_...	Main loop using /grasp/jaw_axis for orientation and active heartbeats.	
run_picker.sh	Bash supervisor that restarts the brain after each successful pick.	
force_release.py	Emergency script to release gripper tension and clear Red LED faults.	
⚠️ Troubleshooting
Red LED Fault: Occurs when the Modbus watchdog is triggered. Ensure active_wait() is used in the script to maintain the heartbeat.

No Communication: Verify socat is running in Terminal 5 and /tmp/ttyUR permissions are set to 666.

Stability Rejection: The brain requires 10 consistent vision frames (spread < 0.02m) before initiating a pick.

Next Step: Would you like me to prepare the robotiq_gripper_node.py refactor tomorrow to finalize the decoupling of Terminal 5 and 6?