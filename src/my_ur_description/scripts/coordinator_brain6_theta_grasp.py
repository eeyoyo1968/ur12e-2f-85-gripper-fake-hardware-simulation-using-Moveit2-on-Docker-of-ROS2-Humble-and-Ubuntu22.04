import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import collections
import numpy as np
import os 
import math
import minimalmodbus 

from rclpy.callback_groups import ReentrantCallbackGroup # Added for parallel processing
from test_move_xyz_theta_noflip_gmove1 import UR12eController

class PickAndPlaceBrain(UR12eController):
    def __init__(self):
        super().__init__()
        # Use a callback group so subscribers aren't blocked by the gripper logic
        self.group = ReentrantCallbackGroup()
        
        self.pose_buffer = collections.deque(maxlen=10)
        self.is_busy = False
        self.current_class = ""
        self.target_class = "unknown"
        self.latest_pose = None

        # --- Gripper Setup ---
        try:
            self.gripper = minimalmodbus.Instrument('/tmp/ttyUR', 9)
            self.gripper.serial.baudrate = 115200
            self.gripper.serial.timeout = 0.1 # Very short to avoid blocking
            self.init_gripper()
        except Exception as e:
            self.get_logger().error(f"Gripper Error: {e}")

        # Subscriptions assigned to the Reentrant Group
        self.obj_pose_sub = self.create_subscription(
            PoseStamped, '/grasp/pose', self.pose_cb, 10, callback_group=self.group)
        self.obj_class_sub = self.create_subscription(
            String, '/grasp/class', self.class_cb, 10, callback_group=self.group)

    def init_gripper(self):
        """Non-blocking activation sequence"""
        self.gripper.write_register(1000, 0x0100)
        # We don't sleep here anymore; we let the main loop handle the wait
        self.get_logger().info("Gripper Activation Sent.")

    def set_gripper(self, position):
        """Corrected 3-register mapping for 2F-85"""
        try:
            # Action (0x0900), Position, Speed/Force (0x6432 = 100/50)
            payload = [0x0900, position, 0x6432]
            self.gripper.write_registers(1000, payload)
        except:
            pass

    def gripper_heartbeat(self):
        """Keep watchdog alive without blocking"""
        try:
            self.gripper.read_register(2000)
        except:
            pass

    # ... [Rest of your helper functions: get_theta, pose_cb, check_stability] ...
    # Ensure check_stability is identical to your working brain5

    def execute_move(self):
        if self.is_busy: return
        self.is_busy = True
        
        # Capture the data so it doesn't change during move
        p = self.latest_pose.position
        theta = self.get_theta_from_pose(self.latest_pose)
        
        # Transformation and Movement Logic
        # (Keep your move_xyz_theta_noflip commands here)
        self.set_gripper(228) # Close 
        # ... robot moves ...
        self.set_gripper(3) # Open
        
        os._exit(0)

def main(args=None):
    rclpy.init(args=args)
    brain = PickAndPlaceBrain()
    
    # Give the gripper time to finish its internal sweep while ROS starts
    time.sleep(4.0) 
    
    print("Waiting for stable vision data...")
    while rclpy.ok():
        # Important: Allow ROS to process callbacks
        rclpy.spin_once(brain, timeout_sec=0.01)
        
        brain.gripper_heartbeat() # Reset watchdog
        
        # Diagnostic print
        if len(brain.pose_buffer) > 0:
            print(f"Tracking object... Buffer: {len(brain.pose_buffer)}/10", end='\r')
        
        if brain.check_stability():
            print("\nStable! Moving...")
            brain.execute_move()

if __name__ == '__main__':
    main()