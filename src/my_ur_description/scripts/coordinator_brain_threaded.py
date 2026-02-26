import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import collections
import numpy as np
import threading

from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from test_move_xyz_theta_noflip_gmove1 import UR12eController

class PickAndPlaceBrain(UR12eController):
    def __init__(self):
        # Create a Mutually Exclusive group to prevent threading crashes
        self.gate_group = MutuallyExclusiveCallbackGroup()
        super().__init__()
        
        self.pose_buffer = collections.deque(maxlen=10) 
        self.stability_threshold = 0.02 
        self.is_busy = False
        self.current_class = ""
        self.target_class = "unknown"
        self.latest_msg = None

        # 1. Subscribers assigned to the MutuallyExclusive group
        self.obj_pose_sub = self.create_subscription(
            PoseStamped, '/grasp/pose', self.pose_cb, 10, callback_group=self.gate_group)
        self.obj_class_sub = self.create_subscription(
            String, '/grasp/class', self.class_cb, 10, callback_group=self.gate_group)

        # 2. Control Loop Timer
        self.timer = self.create_timer(0.2, self.brain_logic, callback_group=self.gate_group)

    def class_cb(self, msg):
        self.current_class = msg.data

    def pose_cb(self, msg):
        self.latest_msg = msg

    def brain_logic(self):
        if self.is_busy or self.latest_msg is None or self.current_class == "":
            return

        p = self.latest_msg.pose.position
        self.pose_buffer.append([p.x, p.y])
        
        if len(self.pose_buffer) >= 10:
            arr = np.array(self.pose_buffer)
            spread = np.max(arr, axis=0) - np.min(arr, axis=0)
            
            if all(spread < self.stability_threshold):
                self.target_class = self.current_class
                self.get_logger().info(f"STABILITY MET! Target: {self.target_class}")
                self.is_busy = True
                # Execute move directly in the timer logic to lock the callback group
                self.execute_sequence(self.latest_msg.pose)

    def execute_sequence(self, pose):
        try:
            x_target = -0.03 - pose.position.x
            y_target = 1.28 - pose.position.y
            z_target = max(0.2275, 0.20 - 0.02 + pose.position.z)

            self.get_logger().info("--- Executing Pick ---")
            self.move_xyz_no_flip(x_target, y_target, z_target + 0.15)
            self.move_xyz_no_flip(x_target, y_target, z_target)
            time.sleep(1.0) 
            self.move_xyz_no_flip(x_target, y_target, z_target + 0.2) 
        
            target_x = 0.2 if self.target_class == "glove" else -0.2
            self.get_logger().info(f"--- Sorting to {self.target_class} bin ---")
            self.move_xyz_no_flip(target_x, 0.4, 0.3)
            time.sleep(1.0)

            self.get_logger().info("--- Returning Home ---")
            self.move_xyz_no_flip(0.0, 0.5, 0.5) 

        except Exception as e:
            self.get_logger().error(f"MOVEMENT FAILED: {e}")
        
        finally:
            self.get_logger().info("Resetting Brain...")
            self.pose_buffer.clear()
            self.latest_msg = None
            self.target_class = "unknown"
            time.sleep(2.0) 
            self.is_busy = False
            self.get_logger().info("SYSTEM READY.")

def main(args=None):
    rclpy.init(args=args)
    brain = PickAndPlaceBrain()
    # Using SingleThreadedExecutor is now the safest option
    executor = SingleThreadedExecutor()
    executor.add_node(brain)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()