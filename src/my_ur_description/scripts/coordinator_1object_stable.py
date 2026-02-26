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
        
        # Original 10-frame buffer for stability
        self.pose_buffer = collections.deque(maxlen=10) 
        self.is_busy = False
        self.current_class = ""

        # Subscribers
        self.create_subscription(PoseStamped, '/grasp/pose', self.detection_callback, 10)
        self.create_subscription(String, '/grasp/class', self.class_callback, 10)

    def class_callback(self, msg):
        self.current_class = msg.data

    def detection_callback(self, msg):
        # Stop looking if we are already in the middle of a move
        if self.is_busy:
            return
        
        # Filtering logic from your original code
        pos = msg.pose.position
        self.pose_buffer.append([pos.x, pos.y])
        
        if len(self.pose_buffer) == 10:
            arr = np.array(self.pose_buffer)
            spread = np.max(arr, axis=0) - np.min(arr, axis=0)
            
            # Only trigger if the object has been stable for 10 frames
            if all(spread < 0.02):
                self.get_logger().info(f"Object Stable: {self.current_class}. Starting Pick.")
                self.is_busy = True
                self.run_one_pick_and_place(msg.pose)

    def run_one_pick_and_place(self, pose):
        try:
            # Workspace Transformations
            x_target = -0.03 - pose.position.x
            y_target = 1.28 - pose.position.y
            z_target = max(0.2275, 0.20 - 0.02 + pose.position.z)

            # --- EXECUTION SEQUENCE ---
            # 1. Approach
            self.move_xyz_no_flip(x_target, y_target, z_target + 0.15)
            # 2. Lower to Object
            self.move_xyz_no_flip(x_target, y_target, z_target)
            time.sleep(1.0) # Wait for grasp
            # 3. Lift
            self.move_xyz_no_flip(x_target, y_target, z_target + 0.2) 
        
            # 4. Bin Sorting
            bin_x = 0.2 if self.current_class == "glove" else -0.2
            self.move_xyz_no_flip(bin_x, 0.4, 0.3)
            time.sleep(1.0) # Wait for release

            # 5. Return Home
            self.move_xyz_no_flip(0.0, 0.5, 0.5) 
            self.get_logger().info("Pick and Place cycle finished successfully.")

        except Exception as e:
            self.get_logger().error(f"Movement failed: {e}")
        
        finally:
            # FORCE TERMINATE: This allows the Bash loop to restart the program
            self.get_logger().info("Terminating script for clean restart...")
            os._exit(0)

def main(args=None):
    rclpy.init(args=args)
    brain = PickAndPlaceBrain()
    executor = SingleThreadedExecutor()
    executor.add_node(brain)
    try:
        executor.spin()
    except KeyboardInterrupt:
        os._exit(0)

if __name__ == '__main__':
    main()