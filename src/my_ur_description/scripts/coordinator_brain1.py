import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import math
import collections
import numpy as np

from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from test_move_xyz_theta_noflip_gmove1 import UR12eController

class PickAndPlaceBrain(UR12eController):
    def __init__(self):
        self.cb_group = ReentrantCallbackGroup()
        super().__init__()
        
        self.pose_buffer = collections.deque(maxlen=10) 
        self.stability_threshold = 0.02 
        self.is_busy = False
        self.current_class = ""
        self.target_class = "unknown"

        # Define Bin and Home Coordinates (Adjust these to your actual workspace)
        self.bin_soft = {"x": 0.25, "y": 0.40, "z": 0.25}
        self.bin_hard = {"x": -0.35, "y": 0.40, "z": 0.25}
        self.home_pos = {"x": 0.0, "y": 0.5, "z": 0.4}

        self.script_pub = self.create_publisher(String, '/ur_script_interface/script_command', 10)
        
        self.obj_pose_sub = self.create_subscription(
            PoseStamped, 
            '/grasp/pose', 
            self.detection_callback, 
            10,
            callback_group=self.cb_group
        )

        self.obj_class_sub = self.create_subscription(
            String, 
            '/grasp/class', 
            self.class_callback, 
            10
        )

    #def class_callback(self, msg):
    #    self.current_class = msg.data
    #    self.target_class = msg.data
    #    self.get_logger().info("Host Brain Online. Waiting for stable /grasp/pose...")
    
    #def class_callback(self, msg):
    #    # Only accept a new class if we are not currently moving
    #    if not self.is_busy:
    #        self.current_class = msg.data
    #        self.target_class = msg.data
    #def class_callback(self, msg):
    #    # Always update the class string, but only set the target 
    #    # if the brain is actually looking for a new object.
    #    self.current_class = msg.data
    #    if not self.is_busy and self.target_class == "unknown":
    #        self.target_class = msg.data
    #        # Log this only once when the target is set to avoid spamming
    #        self.get_logger().info(f"Target re-armed: {self.target_class}")
    
    def class_callback(self, msg):
        # Always keep current_class updated so the bin logic has the latest data
        self.current_class = msg.data 
    
        # Only "re-arm" the target if the robot is idle and waiting
        if not self.is_busy and self.target_class == "unknown":
            self.target_class = msg.data
            self.get_logger().info(f"New Target Armed: {self.target_class}")


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
            # Pass the pose to the sequence function
            self.run_pick_place_sequence(msg.pose)

    def reset_brain(self):
        self.get_logger().info("Resetting brain for next object...")
        self.pose_buffer.clear()
        self.target_class = "unknown"
        # Do NOT reset self.current_class here if you want to use it for 
        # the very next detection immediately.
        time.sleep(2.0) # Longer sleep to ensure the table is clear in the camera's view
        self.is_busy = False

    #def run_pick_place_sequence(self, pose):
    #    self.is_busy = True
    #    
    #    # 1. Transform Calculation (Using your verified math)
    #    x_target = -0.03 - pose.position.x
    #    y_target = 1.13 + 0.15 - pose.position.y
    #    z_target = 0.24 - 0.04 + pose.position.z 
    #
    #    self.get_logger().info(f"Starting Sequence for {self.current_class}")
    #
    #    # --- PICK PHASE ---
    #    # Approach
    #    self.move_xyz_no_flip(x_target, y_target, z_target + 0.15)
    #    # Lower to object
    #    self.move_xyz_no_flip(x_target, y_target, z_target+0.02)
    #    
    #    self.get_logger().info("Object 'Grasped' (Simulated)")
    #    time.sleep(1.0) 
    #
    #    # Lift
    #    self.move_xyz_no_flip(x_target, y_target, z_target + 0.2)
    #
    #    # --- SORTING PHASE ---
    #    if self.current_class == "glove":
    #        target_bin = self.bin_soft
    #        label = "SOFT"
    #    else:
    #        target_bin = self.bin_hard
    #        label = "HARD"
    #
    #    self.get_logger().info(f"Moving to {label} bin...")
    #    
    #    # Move above bin
    #    self.move_xyz_no_flip(target_bin["x"], target_bin["y"], target_bin["z"] + 0.1)
    #    # Lower into bin
    #    self.move_xyz_no_flip(target_bin["x"], target_bin["y"], target_bin["z"])
    #    
    #    self.get_logger().info("Object 'Released' (Simulated)")
    #    time.sleep(1.0)
    #
    #    # Lift out of bin
    #    self.move_xyz_no_flip(target_bin["x"], target_bin["y"], target_bin["z"] + 0.1)
    #
    #    # --- RESET PHASE ---
    #    self.get_logger().info("Returning to Home position.")
    #    self.move_xyz_no_flip(self.home_pos["x"], self.home_pos["y"], self.home_pos["z"])
    #        
    #    self.is_busy = False
    #    self.pose_buffer.clear()

    def run_pick_place_sequence(self, pose):
        self.is_busy = True
    
        # Transformation (Table to Base)
        x_target = -0.03 - pose.position.x
        y_target = 1.28 - pose.position.y
        z_target = 0.20 -0.02 + pose.position.z 

        if z_target <0.2275: # this is to set the lowest point z can reach
           z_target = 0.2275

        # 1. Move Above and Lower (Simulate Grasp)
        self.move_xyz_no_flip(x_target, y_target, z_target + 0.15)
        self.move_xyz_no_flip(x_target, y_target, z_target)
        time.sleep(1.0) 

        # 2. Lift and Transport
        self.move_xyz_no_flip(x_target, y_target, z_target + 0.2) # Lift high
    
        # Choose bin based on class
        target_bin = [0.2, 0.4, 0.3] if self.current_class == "glove" else [-0.2, 0.4, 0.3]
    
        self.get_logger().info(f"Moving to bin for: {self.current_class}")
        self.move_xyz_no_flip(target_bin[0], target_bin[1], target_bin[2])
        time.sleep(1.0) # Simulate Release

        # 3. Return Home
        self.get_logger().info("Returning Home...")
        self.move_xyz_no_flip(0.0, 0.5, 0.5) # Example Home Position
            
        # --- 4. RESET STATE FOR NEXT OBJECT ---
        # Clear the position buffer so the next object starts a fresh stability check
        #self.pose_buffer.clear()
        
        # Reset the target class to force the brain to wait for a NEW detection
        #self.target_class = "unknown"
        
        # IMPORTANT: Add a 2-second delay while is_busy is still True.
        # This prevents the robot from "re-detecting" the object it just moved
        # while it's still physically in the way or the camera hasn't updated.
        #time.sleep(1.0) 
        
        #self.is_busy = False
        #self.get_logger().info("Ready for next object. Waiting for detection...")
        self.get_logger().info("Sequence Complete. Home.")
        self.reset_brain()

def main(args=None):
    rclpy.init(args=args)
    brain = PickAndPlaceBrain()
    
    # Use MultiThreadedExecutor so callbacks can run while the robot is moving
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(brain)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        brain.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()