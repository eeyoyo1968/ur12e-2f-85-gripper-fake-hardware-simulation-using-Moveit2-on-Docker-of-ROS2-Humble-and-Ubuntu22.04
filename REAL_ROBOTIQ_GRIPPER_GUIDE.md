# Complete Guide: Controlling Real Robotiq 2F-85 Gripper

## Overview of Options

You have **3 viable options** for controlling a real Robotiq 2F-85:

```
Option 1: URCap Integration (EASIEST) ⭐ RECOMMENDED
Option 2: Direct Modbus/Serial Control (ADVANCED)
Option 3: Fix ros2_robotiq_gripper (TROUBLESOME)
```

---

## OPTION 1: URCap Integration (RECOMMENDED) ⭐

This is the **easiest and most reliable** method.

### How It Works

```
Robotiq 2F-85 Gripper
    ↓ (Connected to UR tool flange)
UR Controller (with Robotiq URCap)
    ↓ (Ethernet/URScript)
Your ROS2 Action Server
    ↓ (ROS2 Actions)
Your Motion Scripts (unchanged!)
```

### Prerequisites

**Physical Setup:**
1. Gripper connected to UR12e tool flange
2. Power cable from gripper to UR controller
3. Communication cable (comes with gripper)

**Software on UR12e:**
1. Robotiq 2F Gripper URCap installed
2. Gripper activated via URCap interface

### Step 1: Install URCap on Real Robot

**On UR12e Teach Pendant:**

1. **Download Robotiq URCap:**
   - Go to: https://robotiq.com/support
   - Download: `Robotiq_2F_Gripper_URCap_X.X.X.urcap`

2. **Transfer to USB stick**

3. **Install on robot:**
   ```
   Settings → System → URCaps → "+" → Select from USB
   ```

4. **Restart robot**

5. **Verify installation:**
   ```
   Settings → System → URCaps
   Should show: "Robotiq 2-Finger Gripper"
   ```

### Step 2: Configure Gripper

**On UR12e:**

1. **Navigate to:**
   ```
   Installation → URCaps → Robotiq Gripper
   ```

2. **Configure:**
   ```
   Model: 2F-85
   Communication: Tool Communication Interface
   Slave Address: 9 (default)
   ```

3. **Test Gripper:**
   - Click "Activate"
   - Click "Open" - gripper should open
   - Click "Close" - gripper should close

### Step 3: Create ROS2 Action Server

This bridges ROS2 to the UR controller's gripper control.

**File: `/ros2_ws/src/my_ur_description/scripts/robotiq_real_gripper_server.py`**

```python
#!/usr/bin/env python3
"""
ROS2 Action Server for Real Robotiq 2F-85 via UR Controller
Works with both real robot and URSim (with URCap)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
import socket
import time

class RobotiqRealGripperServer(Node):
    def __init__(self):
        super().__init__('robotiq_real_gripper_server')
        
        # Parameters
        self.robot_ip = self.declare_parameter('robot_ip', '192.168.1.100').value
        self.script_port = 30002  # URScript interface
        
        # Gripper state
        self.current_position = 0.0  # 0.0 = open, 0.8 = closed (radians)
        
        # Publisher for gripper state (optional, for monitoring)
        self.state_pub = self.create_publisher(
            JointState,
            '/robotiq_gripper_state',
            10
        )
        
        # Action server - matches your existing controller interface
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/robotiq_gripper_controller/follow_joint_trajectory',
            self.execute_callback
        )
        
        # State publishing timer
        self.timer = self.create_timer(0.1, self.publish_state)
        
        self.get_logger().info(f"Robotiq Real Gripper Server ready (Robot IP: {self.robot_ip})")
        self.get_logger().info("Ensure Robotiq URCap is installed and gripper is activated!")
    
    def send_urscript(self, script):
        """Send URScript commands to UR controller."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2.0)
            s.connect((self.robot_ip, self.script_port))
            
            # Send script
            s.send(script.encode())
            s.close()
            
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to send URScript: {e}")
            return False
    
    def gripper_command(self, position_rad):
        """
        Send gripper command to UR via Robotiq URCap functions.
        position_rad: 0.0 (open) to 0.8 (closed) in radians
        """
        # Convert radians to gripper position (0-255)
        # 0.0 rad = fully open = 0
        # 0.8 rad = fully closed = 255
        gripper_pos = int((position_rad / 0.8) * 255)
        gripper_pos = max(0, min(255, gripper_pos))
        
        self.get_logger().info(f"Commanding gripper to position: {gripper_pos}/255 ({position_rad:.3f} rad)")
        
        # URScript using Robotiq URCap functions
        # These functions are provided by the installed URCap
        ur_script = f"""def gripper_move():
    # Ensure gripper is activated
    rq_activate_and_wait()
    
    # Move to position
    # rq_move_and_wait(position, speed, force)
    # position: 0-255 (0=open, 255=closed)
    # speed: 0-255 (255=max speed)
    # force: 0-255 (255=max force)
    rq_move_and_wait({gripper_pos}, 255, 150)
end

gripper_move()
"""
        
        success = self.send_urscript(ur_script)
        
        if success:
            # Update internal state
            self.current_position = position_rad
        
        return success
    
    def execute_callback(self, goal_handle):
        """Execute gripper trajectory action."""
        request = goal_handle.request
        
        # Extract target position from trajectory
        if len(request.trajectory.points) == 0:
            self.get_logger().error("Empty trajectory received")
            goal_handle.abort()
            return FollowJointTrajectory.Result()
        
        # Get final point
        target_point = request.trajectory.points[-1]
        target_position = target_point.positions[0]  # First joint
        
        self.get_logger().info(f"Executing gripper move to {target_position:.3f} rad")
        
        # Send command
        success = self.gripper_command(target_position)
        
        if not success:
            self.get_logger().error("Failed to command gripper")
            goal_handle.abort()
            return FollowJointTrajectory.Result()
        
        # Wait for movement to complete
        duration = target_point.time_from_start.sec + target_point.time_from_start.nanosec / 1e9
        time.sleep(max(1.0, duration))
        
        # Mark as complete
        goal_handle.succeed()
        
        result = FollowJointTrajectory.Result()
        return result
    
    def publish_state(self):
        """Publish current gripper state."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['robotiq_85_left_knuckle_joint']
        msg.position = [self.current_position]
        msg.velocity = [0.0]
        msg.effort = [0.0]
        
        self.state_pub.publish(msg)

def main():
    rclpy.init()
    server = RobotiqRealGripperServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Install the Script

```bash
cd /ros2_ws/src/my_ur_description

# Make executable
chmod +x scripts/robotiq_real_gripper_server.py

# Update CMakeLists.txt to install it
nano CMakeLists.txt
```

Add to `CMakeLists.txt`:
```cmake
install(PROGRAMS
  scripts/robotiq_real_gripper_server.py
  scripts/robotiq_ursim_bridge.py
  scripts/test_move_xyz_theta_noflip_gmove.py
  DESTINATION lib/${PROJECT_NAME}
)
```

Rebuild:
```bash
cd /ros2_ws
colcon build --packages-select my_ur_description
source install/setup.bash
```

### Step 5: Update Your URDF/Launch

**Keep fake hardware in ur_system.xacro:**
```xml
<!-- Gripper stays as fake in URDF -->
<ros2_control name="RobotiqGripperHardwareInterface" type="system">
  <hardware>
    <plugin>mock_components/GenericSystem</plugin>
  </hardware>
  <!-- ... -->
</ros2_control>
```

**Why?** The action server handles real gripper control externally, so fake hardware in URDF is fine.

### Step 6: Launch Everything

**Terminal 1: Launch Robot System**
```bash
export ROS_DOMAIN_ID=42
source /ros2_ws/install/setup.bash

# Launch main system (robot with fake gripper in URDF)
ros2 launch my_ur_description my_robot.launch.py \
  use_fake_hardware:=false \
  robot_ip:=192.168.1.100 \
  kinematics_file:=/path/to/calibration.yaml

# Press Play on robot teach pendant
```

**Terminal 2: Launch Real Gripper Server**
```bash
export ROS_DOMAIN_ID=42
source /ros2_ws/install/setup.bash

# Launch gripper action server
ros2 run my_ur_description robotiq_real_gripper_server.py --ros-args \
  -p robot_ip:=192.168.1.100
```

### Step 7: Test Gripper

```bash
# Terminal 3: Test gripper commands
export ROS_DOMAIN_ID=42
source /ros2_ws/install/setup.bash

# Open gripper
ros2 action send_goal /robotiq_gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['robotiq_85_left_knuckle_joint'], points: [{positions: [0.0], time_from_start: {sec: 2}}]}}"

# Close gripper
ros2 action send_goal /robotiq_gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: ['robotiq_85_left_knuckle_joint'], points: [{positions: [0.8], time_from_start: {sec: 2}}]}}"
```

**Physical gripper should move!**

### Step 8: Run Your Existing Scripts

```bash
# Your existing motion scripts work WITHOUT changes!
python3 /ros2_ws/src/my_ur_description/scripts/test_move_xyz_theta_noflip_gmove.py
```

The gripper commands in your script (like `bot.gripper_move(0.8)`) now control the **real physical gripper**!

---

## OPTION 2: Direct Modbus Control (Advanced)

If you want direct control without URCap (more complex but more flexible).

### Hardware Setup

Connect gripper via RS-485 (Modbus RTU):
```
Robotiq 2F-85
    ↓ (RS-485 cable)
USB-to-RS485 Adapter
    ↓ (USB)
Your Computer
```

### Install Dependencies

```bash
# Install Modbus library
pip3 install --break-system-packages pymodbus

# Verify USB device
ls -la /dev/ttyUSB*
# Should show: /dev/ttyUSB0 (or similar)

# Set permissions
sudo chmod 666 /dev/ttyUSB0
# Or add user to dialout group:
sudo usermod -a -G dialout $USER
# (logout and login after this)
```

### Create Modbus Driver

**File: `/ros2_ws/src/my_ur_description/scripts/robotiq_modbus_server.py`**

```python
#!/usr/bin/env python3
"""
Direct Modbus RTU control of Robotiq 2F-85
Requires physical RS-485 connection
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from pymodbus.client import ModbusSerialClient
import time

class RobotiqModbusServer(Node):
    def __init__(self):
        super().__init__('robotiq_modbus_server')
        
        # Parameters
        port = self.declare_parameter('port', '/dev/ttyUSB0').value
        baudrate = self.declare_parameter('baudrate', 115200).value
        slave_id = self.declare_parameter('slave_id', 9).value
        
        # Modbus connection
        self.client = ModbusSerialClient(
            port=port,
            baudrate=baudrate,
            timeout=1,
            parity='N',
            stopbits=1,
            bytesize=8
        )
        
        if not self.client.connect():
            self.get_logger().error(f"Failed to connect to gripper on {port}")
            return
        
        self.get_logger().info(f"Connected to Robotiq gripper on {port}")
        self.slave_id = slave_id
        
        # Gripper state
        self.current_position = 0.0
        
        # Initialize gripper
        self.activate_gripper()
        
        # State publisher
        self.state_pub = self.create_publisher(JointState, '/robotiq_gripper_state', 10)
        
        # Action server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/robotiq_gripper_controller/follow_joint_trajectory',
            self.execute_callback
        )
        
        # State timer
        self.timer = self.create_timer(0.1, self.publish_state)
    
    def activate_gripper(self):
        """Activate the gripper (required on startup)."""
        self.get_logger().info("Activating gripper...")
        
        # Write activation command to register 0x03E8
        # Format: rACT=1, rGTO=0, rATR=0
        self.client.write_register(0x03E8, 0x0100, slave=self.slave_id)
        time.sleep(0.5)
        
        # Wait for activation
        for i in range(20):
            response = self.client.read_holding_registers(0x07D0, 1, slave=self.slave_id)
            if response.isError():
                continue
            
            status = response.registers[0]
            # Check activation bit (gACT)
            if (status & 0x0100) == 0x0100:
                self.get_logger().info("Gripper activated!")
                return True
            
            time.sleep(0.5)
        
        self.get_logger().error("Gripper activation failed")
        return False
    
    def gripper_command(self, position_rad):
        """Move gripper to position."""
        # Convert radians to gripper units (0-255)
        gripper_pos = int((position_rad / 0.8) * 255)
        gripper_pos = max(0, min(255, gripper_pos))
        
        self.get_logger().info(f"Moving to position: {gripper_pos}/255")
        
        # Write command
        # Register 0x03E9: rACT=1, rGTO=1 (go to position)
        self.client.write_register(0x03E9, 0x0900, slave=self.slave_id)
        
        # Register 0x03EA: Position (0-255)
        self.client.write_register(0x03EA, gripper_pos, slave=self.slave_id)
        
        # Register 0x03EB: Speed (0-255)
        self.client.write_register(0x03EB, 255, slave=self.slave_id)
        
        # Register 0x03EC: Force (0-255)
        self.client.write_register(0x03EC, 150, slave=self.slave_id)
        
        # Wait for completion
        timeout = 5.0
        start_time = time.time()
        while time.time() - start_time < timeout:
            response = self.client.read_holding_registers(0x07D0, 3, slave=self.slave_id)
            if response.isError():
                continue
            
            status = response.registers[0]
            # Check if motion complete (gOBJ bits)
            if (status & 0xC000) != 0x0000:
                break
            
            time.sleep(0.1)
        
        # Read final position
        response = self.client.read_holding_registers(0x07D1, 1, slave=self.slave_id)
        if not response.isError():
            actual_pos = response.registers[0]
            self.current_position = (actual_pos / 255.0) * 0.8
            self.get_logger().info(f"Final position: {actual_pos}/255")
        
        return True
    
    def execute_callback(self, goal_handle):
        """Execute gripper trajectory."""
        request = goal_handle.request
        
        if len(request.trajectory.points) == 0:
            goal_handle.abort()
            return FollowJointTrajectory.Result()
        
        target_point = request.trajectory.points[-1]
        target_position = target_point.positions[0]
        
        success = self.gripper_command(target_position)
        
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        
        return FollowJointTrajectory.Result()
    
    def publish_state(self):
        """Publish gripper state."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['robotiq_85_left_knuckle_joint']
        msg.position = [self.current_position]
        msg.velocity = [0.0]
        msg.effort = [0.0]
        
        self.state_pub.publish(msg)

def main():
    rclpy.init()
    server = RobotiqModbusServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Launch Modbus Driver

```bash
# Make executable
chmod +x scripts/robotiq_modbus_server.py

# Run
ros2 run my_ur_description robotiq_modbus_server.py --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p baudrate:=115200 \
  -p slave_id:=9
```

---

## OPTION 3: Fix ros2_robotiq_gripper (Not Recommended)

The package you have is looking for an old "serial" library. You can try to fix it, but it's more work.

### Try Building Custom Serial Library

```bash
cd /ros2_ws/src

# Clone serial library
git clone https://github.com/wjwwood/serial.git

# Build
cd serial
make
sudo make install

# Try building robotiq again
cd /ros2_ws
colcon build --packages-select robotiq_driver
```

**However**, this is more trouble than it's worth. Use Option 1 or 2 instead.

---

## Comparison of Options

| Feature | URCap (Option 1) | Modbus (Option 2) | Fix Package (Option 3) |
|---------|------------------|-------------------|------------------------|
| Ease of Setup | ⭐⭐⭐⭐⭐ Easy | ⭐⭐⭐ Medium | ⭐ Hard |
| Reliability | ⭐⭐⭐⭐⭐ High | ⭐⭐⭐⭐ High | ⭐⭐ Variable |
| Hardware Needed | Just gripper on robot | USB-RS485 adapter | Multiple attempts |
| Code Changes | None | None | Unknown |
| Recommended | ✅ YES | For advanced users | ❌ NO |

---

## Recommended Workflow

**Phase 1: URCap Setup (Today)**
1. Install Robotiq URCap on UR12e
2. Test gripper via teach pendant
3. Deploy `robotiq_real_gripper_server.py`
4. Test with ROS2 commands
5. Run your existing motion scripts

**Phase 2: Integration (Next)**
1. Test pick-and-place with real gripper
2. Tune gripper force/speed if needed
3. Add force feedback (optional)
4. Deploy full vision-guided grasping

---

## Complete Launch Script

**File: `launch_real_system.sh`**

```bash
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
```

---

## Summary

**Recommended Approach: URCap Integration (Option 1)**

✅ **What you need:**
1. Robotiq URCap installed on UR12e
2. `robotiq_real_gripper_server.py` script (provided above)
3. Same launch file, no URDF changes

✅ **What you get:**
- Real gripper control via ROS2
- No code changes in your motion scripts
- Same action interface as before
- Reliable, production-ready solution

✅ **Setup time:** ~30 minutes

**Your existing code works without ANY changes!** 🎯

Which option would you like to implement? I recommend starting with Option 1 (URCap).
