import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import collections
import numpy as np
import os 
import math
import minimalmodbus 

from rclpy.callback_groups import ReentrantCallbackGroup
from test_move_xyz_theta_noflip_gmove1 import UR12eController

class PickAndPlaceBrain(UR12eController):
    def __init__(self):
        super().__init__()
        self.group = ReentrantCallbackGroup()
        self.pose_buffer = collections.deque(maxlen=10)
        self.is_busy = False
        self.current_class = ""
        self.target_class = "unknown"
        self.latest_pose = None
        self.gripper_connected = False

        # Physical Constants from your Diagnostic
        self.GRIPPER_OPEN = 3
        self.GRIPPER_CLOSED = 228 

        # --- Fast Gripper Connection ---
        try:
            self.gripper = minimalmodbus.Instrument('/tmp/ttyUR', 9)
            self.gripper.serial.baudrate = 115200
            self.gripper.serial.timeout = 0.1 
            self.gripper.write_register(1000, 0x0100) # Activate
            self.gripper_connected = True
        except Exception as e:
            self.get_logger().error(f"Gripper Init Failed: {e}")

        self.obj_pose_sub = self.create_subscription(
            PoseStamped, '/grasp/pose', self.pose_cb, 10, callback_group=self.group)
        self.obj_class_sub = self.create_subscription(
            String, '/grasp/class', self.class_cb, 10, callback_group=self.group)

    def pose_cb(self, msg):
        if not self.is_busy:
            self.latest_pose = msg.pose

    def class_cb(self, msg):
        if not self.is_busy:
            self.current_class = msg.data
            self.target_class = msg.data

    def set_gripper(self, position):
        """Standard 3-register mapping"""
        if not self.gripper_connected: return
        try:
            # Action, Position, Speed/Force
            self.gripper.write_registers(1000, [0x0900, position, 0x6432])
        except:
            pass

    def gripper_heartbeat(self):
        if not self.gripper_connected: return
        try:
            self.gripper.read_register(2000)
        except:
            pass

    def get_theta_from_pose(self, pose):
        q = pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def check_stability(self):
        if self.latest_pose is None or self.latest_pose.position.x == 0.0:
            return False
        self.pose_buffer.append([self.latest_pose.position.x, self.latest_pose.position.y])
        if len(self.pose_buffer) < 10: return False
        arr = np.array(self.pose_buffer)
        spread = np.max(arr, axis=0) - np.min(arr, axis=0)
        return all(spread < 0.02)

    def execute_move(self):
        self.is_busy = True
        p = self.latest_pose.position
        theta = self.get_theta_from_pose(self.latest_pose)
        
        # Transformation Logic
        x_t, y_t = -0.03 - p.x, 1.28 - p.y
        z_t = max(0.2275, 0.20 - 0.02 + p.z)

        self.get_logger().info(f"MOVING NOW: {self.target_class}")
        
        try:
            # CRITICAL: Move Arm FIRST to lock the External Control thread
            self.move_xyz_theta_noflip(x_t, y_t, z_t + 0.15, theta)
            
            # Now handle gripper while moving or paused
            self.set_gripper(self.GRIPPER_OPEN)
            self.move_xyz_theta_noflip(x_t, y_t, z_t, theta)
            
            time.sleep(0.5)
            self.set_gripper(self.GRIPPER_CLOSED)
            time.sleep(1.5) 
            
            self.move_xyz_theta_noflip(x_t, y_t, z_t + 0.2, theta)
            
            bin_x = 0.2 if self.current_class == "glove" else -0.2
            self.move_xyz_theta_noflip(bin_x, 0.4, 0.3, 0.0)
            
            self.set_gripper(self.GRIPPER_OPEN)
            time.sleep(1.0)
            self.move_xyz_theta_noflip(0.0, 0.5, 0.5, 0.0)
            
        finally:
            print("Sequence Finished.")
            os._exit(0)

def main(args=None):
    rclpy.init(args=args)
    brain = PickAndPlaceBrain()
    time.sleep(4.0) # Wait for gripper sweep
    
    while rclpy.ok():
        rclpy.spin_once(brain, timeout_sec=0.01)
        brain.gripper_heartbeat()
        if brain.check_stability():
            brain.execute_move()

if __name__ == '__main__':
    main()