import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import collections
import numpy as np
import os
import sys

from rclpy.executors import SingleThreadedExecutor
from test_move_xyz_theta_noflip_gmove1 import UR12eController

class PickAndPlaceSingle(UR12eController):
    def __init__(self):
        super().__init__()
        
        self.pose_buffer = collections.deque(maxlen=5) # Reduced to 5 for faster detection
        self.is_busy = False
        self.latest_msg = None
        self.current_class = ""

        # Subscribers
        self.obj_pose_sub = self.create_subscription(PoseStamped, '/grasp/pose', self.pose_cb, 10)
        self.obj_class_sub = self.create_subscription(String, '/grasp/class', self.class_cb, 10)

        # Faster check rate (0.1s instead of 0.2s)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Searching for object...")

    def class_cb(self, msg):
        self.current_class = msg.data

    def pose_cb(self, msg):
        self.latest_msg = msg

    def control_loop(self):
        if self.is_busy or self.latest_msg is None or self.current_class == "":
            return

        p = self.latest_msg.pose.position
        self.pose_buffer.append([p.x, p.y])
        
        if len(self.pose_buffer) >= 5:
            arr = np.array(self.pose_buffer)
            spread = np.max(arr, axis=0) - np.min(arr, axis=0)
            
            # If stable enough (within 2cm), go immediately
            if all(spread < 0.02):
                self.get_logger().info(f"STABLE: {self.current_class}. Moving NOW.")
                self.is_busy = True
                self.execute_one_cycle(self.latest_msg.pose)

    def execute_one_cycle(self, pose):
        try:
            # IK Targets
            x_t = -0.03 - pose.position.x
            y_t = 1.28 - pose.position.y
            z_t = max(0.2275, 0.20 - 0.02 + pose.position.z)

            # Sequence with NO extra sleeps
            self.move_xyz_no_flip(x_t, y_t, z_t + 0.15) # Approach
            self.move_xyz_no_flip(x_t, y_t, z_t)        # Pick
            self.move_xyz_no_flip(x_t, y_t, z_t + 0.2)  # Lift
        
            # Bin Sort
            target_x = 0.2 if self.current_class == "glove" else -0.2
            self.move_xyz_no_flip(target_x, 0.4, 0.3)

            # Home
            self.move_xyz_no_flip(0.0, 0.5, 0.5) 
            self.get_logger().info("Done. Terminating.")

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        
        finally:
            # Force quit the process immediately to trigger the Bash restart
            os._exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceSingle()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        os._exit(0)

if __name__ == '__main__':
    main()