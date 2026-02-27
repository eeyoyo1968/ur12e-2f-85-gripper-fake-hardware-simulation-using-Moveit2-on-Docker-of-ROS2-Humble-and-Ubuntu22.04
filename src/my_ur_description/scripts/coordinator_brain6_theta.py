import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import collections
import numpy as np
import os 
import math
import minimalmodbus  # Use this for direct serial communication

from rclpy.executors import SingleThreadedExecutor
from test_move_xyz_theta_noflip_gmove1 import UR12eController

class PickAndPlaceBrain(UR12eController):
    def __init__(self):
        super().__init__()
        self.pose_buffer = collections.deque(maxlen=10)
        self.is_busy = False
        self.current_class = ""
        self.target_class = "unknown"
        self.latest_pose = None

        # Gripper Setup via Tool Communication Forwarder
        try:
            self.gripper = minimalmodbus.Instrument('/tmp/ttyUR', 9) # Slave address 9 is default for Robotiq
            self.gripper.serial.baudrate = 115200
            self.gripper.serial.timeout = 0.5
            self.init_gripper()
            self.get_logger().info("Gripper initialized on /tmp/ttyUR")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to gripper: {e}")

        self.obj_pose_sub = self.create_subscription(PoseStamped, '/grasp/pose', self.pose_cb, 10)
        self.obj_class_sub = self.create_subscription(String, '/grasp/class', self.class_cb, 10)

    def init_gripper(self):
        """Activation sequence for Robotiq 2F-85"""
        # Clear faults and reset
        self.gripper.write_registers(1000, [0x0000, 0x0000, 0x0000])
        time.sleep(0.1)
        # Activate gripper
        self.gripper.write_register(1000, 0x0100)
        time.sleep(1.0)

    def set_gripper(self, position):
        """0 is fully open, 255 is fully closed"""
        # Register 1000: Action Request, Register 1001: Reserved/Force, Register 1002: Position
        # We send 0x09 to Register 1000 to enable 'Go to Position'
        self.gripper.write_registers(1000, [0x0900, 0x0000, position << 8])

    def get_theta_from_pose(self, pose):
        """Extract yaw from quaternion"""
        q = pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def class_cb(self, msg):
        if not self.is_busy:
            self.current_class = msg.data
            self.target_class = msg.data

    def pose_cb(self, msg):
        if not self.is_busy:
            self.latest_pose = msg.pose

    def check_stability(self):
        if self.latest_pose is None:
            return False
        if self.latest_pose.position.x == 0.0 and self.latest_pose.position.y == 0.0:
            return False # Prevent IK Error

        self.pose_buffer.append([self.latest_pose.position.x, self.latest_pose.position.y])
        if len(self.pose_buffer) < 10:
            return False
        
        arr = np.array(self.pose_buffer)
        spread = np.max(arr, axis=0) - np.min(arr, axis=0)
        return all(spread < 0.02)

    def execute_move(self):
        self.is_busy = True
        p = self.latest_pose.position
        theta = self.get_theta_from_pose(self.latest_pose)
        
        try:
            # Transformation logic
            x_t = -0.03 - p.x
            y_t = 1.28 - p.y
            z_t = max(0.2275, 0.20 - 0.02 + p.z)

            self.get_logger().info(f"Target: {self.target_class} | Theta: {math.degrees(theta):.1f}")
            
            # --- START SEQUENCE ---
            self.set_gripper(0) # Open
            
            self.move_xyz_theta_noflip(x_t, y_t, z_t + 0.15, theta)
            self.move_xyz_theta_noflip(x_t, y_t, z_t, theta)
            
            time.sleep(0.5)
            self.set_gripper(255) # Close
            time.sleep(1.0) # Wait for firm grasp
            
            self.move_xyz_theta_noflip(x_t, y_t, z_t + 0.2, theta)
            
            bin_x = 0.2 if self.current_class == "glove" else -0.2
            self.move_xyz_theta_noflip(bin_x, 0.4, 0.3, 0.0)
            
            self.set_gripper(0) # Release
            time.sleep(0.5)
            
            self.move_xyz_theta_noflip(0.0, 0.5, 0.5, 0.0) # Home
        finally:
            os._exit(0) # Exit for Bash loop restart

def main(args=None):
    rclpy.init(args=args)
    brain = PickAndPlaceBrain()
    
    while rclpy.ok():
        rclpy.spin_once(brain, timeout_sec=0.1) # Sequential execution
        if brain.check_stability():
            brain.execute_move()

if __name__ == '__main__':
    main()