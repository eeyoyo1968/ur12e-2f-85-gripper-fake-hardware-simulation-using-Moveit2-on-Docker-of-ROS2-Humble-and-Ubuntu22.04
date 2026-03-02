import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import collections
import numpy as np
import os 
import math # Added for atan2 and degrees

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

    def get_theta_from_pose(self, pose):
        """Extracts the yaw (theta) from the orientation quaternion."""
        q = pose.orientation
        # Conversion from quaternion to Euler yaw (Z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        
        # Depending on your move_xyz_theta_noflip implementation, 
        # you might need radians or degrees. Most MoveIt scripts use radians.
        return yaw_rad

    def check_stability(self):
        if self.latest_pose is None:
            return False
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
        
        # Calculate theta for gripper alignment
        theta = self.get_theta_from_pose(self.latest_pose)
        
        
        try:
            # Table to Base Transformation
            x_t = -0.03 - p.x
            y_t = 1.28 - p.y
            z_t = max(0.2275, 0.20 - 0.02 + p.z)

            self.get_logger().info(f"Target: {self.target_class} at Theta: {math.degrees(theta):.2f}°")

            # 1. Approach with Theta
            self.move_xyz_theta_no_flip(x_t, y_t, z_t + 0.15, theta)
            # 2. Lower to Pick
            self.move_xyz_theta_no_flip(x_t, y_t, z_t, theta)
            time.sleep(1.0) # Simulation of gripper closing
            # 3. Lift
            self.move_xyz_theta_no_flip(x_t, y_t, z_t + 0.2, theta)
            
            # 4. Bin Sort (Orientation usually reset to 0 or 90 for the bin)
            bin_x = 0.2 if self.current_class == "glove" else -0.2
            self.move_xyz_theta_no_flip(bin_x, 0.4, 0.3, 0.0)

            
            
            # 5. Return Home
            self.move_xyz_theta_no_flip(0.0, 0.5, 0.5, 0.0)
            
        finally:
            os._exit(0)

def main(args=None):
    rclpy.init(args=args)
    brain = PickAndPlaceBrain()
    
    while rclpy.ok():
        rclpy.spin_once(brain, timeout_sec=0.1)
        if brain.check_stability():
            brain.get_logger().info("Stability met. Starting Orientated Pick.")
            brain.execute_move()

if __name__ == '__main__':
    main()