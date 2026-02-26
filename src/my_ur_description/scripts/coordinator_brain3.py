import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import collections
import numpy as np
import threading 
import os # - Added to handle immediate process termination

from rclpy.executors import SingleThreadedExecutor
from test_move_xyz_theta_noflip_gmove1 import UR12eController

class PickAndPlaceBrain(UR12eController):
    def __init__(self):
        super().__init__()
        
        self.pose_buffer = collections.deque(maxlen=10) #
        self.stability_threshold = 0.02 #
        self.is_busy = False #
        self.current_class = "" #
        self.target_class = "unknown" #

        self.obj_pose_sub = self.create_subscription(
            PoseStamped, '/grasp/pose', self.detection_callback, 10) #

        self.obj_class_sub = self.create_subscription(
            String, '/grasp/class', self.class_callback, 10) #

    def class_callback(self, msg):
        self.current_class = msg.data #
        if not self.is_busy and self.target_class == "unknown":
            self.target_class = msg.data #
            self.get_logger().info(f"New Target Armed: {self.target_class}") #

    def detection_callback(self, msg):
        if self.is_busy or self.target_class == "unknown":
            return #
        
        self.pose_buffer.append([msg.pose.position.x, msg.pose.position.y]) #
        if len(self.pose_buffer) < 10:
            return #
        
        arr = np.array(self.pose_buffer) #
        spread = np.max(arr, axis=0) - np.min(arr, axis=0) #
        
        if all(spread < self.stability_threshold): #
            self.get_logger().info(f"Stability Met! Target: {self.target_class}") #
            self.is_busy = True #
            thread = threading.Thread(target=self.run_pick_place_sequence, args=(msg.pose,)) #
            thread.start() #

    def run_pick_place_sequence(self, pose):
        try:
            # Transformation (Table to Base)
            x_target = -0.03 - pose.position.x #
            y_target = 1.28 - pose.position.y #
            z_target = max(0.2275, 0.20 - 0.02 + pose.position.z) #

            # 1. PICK PHASE
            self.move_xyz_no_flip(x_target, y_target, z_target + 0.15) #
            self.move_xyz_no_flip(x_target, y_target, z_target) #
            time.sleep(1.0) #

            # 2. TRANSPORT PHASE
            self.move_xyz_no_flip(x_target, y_target, z_target + 0.2) #
        
            if self.current_class == "glove": #
                self.get_logger().info("Moving to SOFT bin") #
                target_bin = [0.2, 0.4, 0.3] #
            else:
                self.get_logger().info(f"Object is {self.current_class}. Moving to HARD bin") #
                target_bin = [-0.2, 0.4, 0.3] #
        
            self.move_xyz_no_flip(target_bin[0], target_bin[1], target_bin[2]) #
            time.sleep(1.0) #

            # 3. RETURN HOME
            self.get_logger().info("Returning Home...") #
            self.move_xyz_no_flip(0.0, 0.5, 0.5) #

        finally:
            # 4. TERMINATE IMMEDIATELY
            self.get_logger().info("Cycle complete. Exiting for clean restart...") #
            # os._exit bypasses Python/ROS cleanup hangs and kills the process instantly
            os._exit(0) #

def main(args=None):
    rclpy.init(args=args) #
    brain = PickAndPlaceBrain() #
    executor = SingleThreadedExecutor() #
    executor.add_node(brain) #
    executor.spin() #
    rclpy.shutdown() #

if __name__ == '__main__':
    main() #