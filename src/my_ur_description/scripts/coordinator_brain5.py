import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import collections
import numpy as np
import os 

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

        self.obj_pose_sub = self.create_subscription(PoseStamped, '/grasp/pose', self.pose_cb, 10)
        self.obj_class_sub = self.create_subscription(String, '/grasp/class', self.class_cb, 10)

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
        
        # Prevent IK Error: Ignore poses that are exactly at origin (sensor noise)
        if self.latest_pose.position.x == 0.0 and self.latest_pose.position.y == 0.0:
            return False

        self.pose_buffer.append([self.latest_pose.position.x, self.latest_pose.position.y])
        if len(self.pose_buffer) < 10:
            return False
        
        arr = np.array(self.pose_buffer)
        spread = np.max(arr, axis=0) - np.min(arr, axis=0)
        return all(spread < 0.02)

    def execute_move(self):
        self.is_busy = True
        p = self.latest_pose.position
        try:
            # Transformation logic
            x_t = -0.03 - p.x
            y_t = 1.28 - p.y
            z_t = max(0.2275, 0.20 - 0.02 + p.z)

            self.get_logger().info(f"MOVING TO TARGET: {self.target_class}")
            self.move_xyz_no_flip(x_t, y_t, z_t + 0.15)
            self.move_xyz_no_flip(x_t, y_t, z_t)
            time.sleep(0.5)
            self.move_xyz_no_flip(x_t, y_t, z_t + 0.2)
            
            bin_x = 0.2 if self.current_class == "glove" else -0.2
            self.move_xyz_no_flip(bin_x, 0.4, 0.3)
            
            self.move_xyz_no_flip(0.0, 0.5, 0.5) # Home
        finally:
            os._exit(0) # Force immediate exit for Bash loop restart

def main(args=None):
    rclpy.init(args=args)
    brain = PickAndPlaceBrain()
    
    # Strictly Sequential Execution Loop
    while rclpy.ok():
        # 1. Look for object (one frame at a time)
        rclpy.spin_once(brain, timeout_sec=0.1)
        
        # 2. If stable, break perception and start movement
        if brain.check_stability():
            brain.get_logger().info("Stability met. Starting Move.")
            brain.execute_move()

if __name__ == '__main__':
    main()