import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import collections
import numpy as np
import threading 
import os 

from rclpy.executors import SingleThreadedExecutor
from test_move_xyz_theta_noflip_gmove1 import UR12eController

class PickAndPlaceBrain(UR12eController):
    def __init__(self):
        super().__init__()
        
        self.pose_buffer = collections.deque(maxlen=10) 
        self.stability_threshold = 0.02 
        self.is_busy = False 
        self.current_class = "" 
        self.target_class = "unknown" 

        # We will create/destroy these to prevent the RCLError
        self.start_subscriptions()

    def start_subscriptions(self):
        """Enable sensor data collection"""
        self.obj_pose_sub = self.create_subscription(
            PoseStamped, '/grasp/pose', self.detection_callback, 10)
        self.obj_class_sub = self.create_subscription(
            String, '/grasp/class', self.class_callback, 10)

    def stop_subscriptions(self):
        """CRITICAL: Destroy subscriptions to prevent RCLError during movement"""
        self.destroy_subscription(self.obj_pose_sub)
        self.destroy_subscription(self.obj_class_sub)

    def class_callback(self, msg):
        self.current_class = msg.data 
        if not self.is_busy and self.target_class == "unknown":
            self.target_class = msg.data 

    def detection_callback(self, msg):
        if self.is_busy or self.target_class == "unknown":
            return 
        
        self.pose_buffer.append([msg.pose.position.x, msg.pose.position.y]) 
        if len(self.pose_buffer) < 10:
            return 
        
        arr = np.array(self.pose_buffer) 
        spread = np.max(arr, axis=0) - np.min(arr, axis=0) 
        
        if all(spread < self.stability_threshold): 
            self.get_logger().info(f"Stability Met! Target: {self.target_class}") 
            self.is_busy = True 
            
            # STOP subscriptions BEFORE starting the move thread
            self.stop_subscriptions()
            
            thread = threading.Thread(target=self.run_pick_place_sequence, args=(msg.pose,)) 
            thread.start() 

    def run_pick_place_sequence(self, pose):
        try:
            # Transformation logic
            x_target = -0.03 - pose.position.x 
            y_target = 1.28 - pose.position.y 
            z_target = max(0.2275, 0.20 - 0.02 + pose.position.z) 

            # Movement with the Action Client is now safe because subs are stopped
            self.move_xyz_no_flip(x_target, y_target, z_target + 0.15) 
            self.move_xyz_no_flip(x_target, y_target, z_target) 
            time.sleep(1.0) 
            self.move_xyz_no_flip(x_target, y_target, z_target + 0.2) 
        
            target_bin = [0.2, 0.4, 0.3] if self.current_class == "glove" else [-0.2, 0.4, 0.3]
            self.move_xyz_no_flip(target_bin[0], target_bin[1], target_bin[2]) 
            time.sleep(1.0) 

            self.get_logger().info("Returning Home...") 
            self.move_xyz_no_flip(0.0, 0.5, 0.5) 

        except Exception as e:
            self.get_logger().error(f"Move Error: {e}")
        finally:
            self.get_logger().info("Cycle complete. Exiting...") 
            os._exit(0) 

def main(args=None):
    rclpy.init(args=args) 
    brain = PickAndPlaceBrain() 
    executor = SingleThreadedExecutor() 
    executor.add_node(brain) 
    executor.spin() 
    rclpy.shutdown() 

if __name__ == '__main__':
    main()