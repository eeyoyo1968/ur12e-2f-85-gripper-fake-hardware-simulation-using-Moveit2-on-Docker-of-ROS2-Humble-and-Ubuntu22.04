import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import math
import collections
import numpy as np

from rclpy.executors import MultiThreadedExecutor

from rclpy.callback_groups import ReentrantCallbackGroup

# This assumes test_move_xyz_theta_noflip_gmove1.py is in the same folder or in your PYTHONPATH
from test_move_xyz_theta_noflip_gmove1 import UR12eController

class PickAndPlaceBrain(UR12eController):
    def __init__(self):
        self.cb_group = ReentrantCallbackGroup()
        super().__init__()
        
        # 1. Stability Buffer (Windows size 10, threshold 2cm)
        self.pose_buffer = collections.deque(maxlen=10) 
        self.stability_threshold = 0.02 
        self.is_busy = False
        
        # 2. URScript Publisher for Wrist Gripper
        self.script_pub = self.create_publisher(String, '/ur_script_interface/script_command', 10)
        
        # 3. Subscribers
        # Update your subscriber to use this group
        self.obj_pose_sub = self.create_subscription(
            PoseStamped, 
            '/grasp/pose', 
            self.detection_callback, 
            10,
            callback_group=self.cb_group
        )

        self.obj_class_sub = self.create_subscription(String, '/grasp/class', self.class_callback, 10)

        self.obj_class_sub = self.create_subscription(String, '/grasp/class', self.class_callback, 10)
        self.current_class = ""




    # ADD THIS METHOD:
    def class_callback(self, msg):
        self.current_class = msg.data
        # self.get_logger().info(f"Detected object class: {self.current_class}")

        self.target_class = msg.data # Change this from "unknown" to the received data
        #self.target_class = "unknown"
        self.get_logger().info("Host Brain Online. Waiting for stable /grasp/pose...")

    #def init_wrist_communication(self):
    #    """Initializes the UR12e wrist pins for the 2F-85"""
    #    msg = String()
    #    msg.data = "set_tool_voltage(24)\nset_tool_communication(True, 115200, 0, 1, 1.5, 3.5)\n"
    #    self.script_pub.publish(msg)

    def init_wrist_communication(self):
        """Initializes the UR12e wrist pins and ensures 24V power is active"""
        msg = String()
        # Adding a small delay and ensuring tool voltage is set before comms
        msg.data = (
            "set_tool_voltage(24)\n"
            "sleep(0.5)\n"
            "set_tool_communication(True, 115200, 0, 1, 1.5, 3.5)\n"
        )
        self.script_pub.publish(msg)
        self.get_logger().info("Sent Tool Power and Communication Init")

    def detection_callback(self, msg):
        if self.is_busy or self.target_class == "unknown":
            return
        
        # Stability Check
        self.pose_buffer.append([msg.pose.position.x, msg.pose.position.y])
        if len(self.pose_buffer) < 10:
            return
        
        arr = np.array(self.pose_buffer)
        spread = np.max(arr, axis=0) - np.min(arr, axis=0)
        
        if all(spread < self.stability_threshold):
            self.get_logger().info(f"Stability Met! Target: {self.target_class}")
            self.run_pick_place_sequence(msg.pose)

    def gripper_wrist_move(self, pos_0_to_255):
        """Sends rq_move_and_wait to wrist pins"""
        script = String()
        script.data = f"rq_move_and_wait({int(pos_0_to_255)})\n"
        self.script_pub.publish(script)
        time.sleep(1.2)

    def run_pick_place_sequence(self, pose):
        self.is_busy = True
        self.init_wrist_communication()
        
        # Base Transformation logic
        #x_base = -pose.position.x
        #y_base = 1.13 + pose.position.y
        #z_base = pose.position.z + 0.15 
    
        x_base = -0.03 - pose.position.x
        y_base = 1.13 +0.15 - pose.position.y
        z_base = 0.24-0.04 + pose.position.z 
    

        # Cycle Execution
        # Approach
        result = self.move_xyz_no_flip(x_base, y_base, z_base + 0.1)
        if result is None:
            self.get_logger().error("Approach motion failed - result is None")
            self.is_busy = False
            return 
    
        # Gripper Open
        self.get_logger().info("Requesting Gripper OPEN")
        #self.gripper_wrist_move(0) # 0 is fully open
    
        # Lower
        self.move_xyz_no_flip(x_base, y_base, z_base) 
    
        # Gripper Close
        self.get_logger().info("Requesting Gripper CLOSE")
        #self.gripper_wrist_move(255) # 255 is fully closed
        
        #if self.monitor_grasp(): 
        #    self.get_logger().info("Grasp success! Moving to bin...")
            # Sorting logic would go here
            
        self.is_busy = False
        self.pose_buffer.clear()

    #def run_pick_place_sequence(self):
    #    self.is_busy = True
    #    p = self.target_pose.pose.position
    # 
    #    # Use the frame provided by the perception node
    #    frame = self.target_pose.header.frame_id # 'table_world'
    #    # In coordinator_brain.py inside brain_pick_and_place:
    #
    #    self.get_logger().info(f"Targeting {self.target_class} in {frame}")
    #
    #    # A. Pre-Grasp (Pass the frame_id to MoveIt) 
    #    self.move_xyz_no_flip(p.x, p.y, p.z + 0.1, frame_id=frame)
    #    self.gripper_move(0.0)
    # 
    #    # B. Grasp
    #    self.move_xyz_no_flip(p.x, p.y, p.z, frame_id=frame)
    #    # ... rest of your logic ...
    #    self.gripper_wrist_move(0)                          # Open
    #    self.move_xyz_no_flip(x_base, y_base, z_base)       # Lower
    #    self.gripper_wrist_move(255)                        # Close
    #    
    #    if self.monitor_grasp(): 
    #        self.get_logger().info("Grasp success! Moving to bin...")
    #        # Sorting logic would go here
    #        
    #    self.is_busy = False
    #    self.pose_buffer.clear()

def main(args=None):
    rclpy.init(args=args)
    brain = PickAndPlaceBrain()
    
    # Use MultiThreadedExecutor to prevent the callback deadlock
    #executor = MultiThreadedExecutor()
    executor = rclpy.executors.SingleThreadedExecutor()
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