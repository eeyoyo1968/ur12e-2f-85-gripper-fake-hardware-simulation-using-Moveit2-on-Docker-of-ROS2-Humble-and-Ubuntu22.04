import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import String
import time
import collections
import numpy as np
import os 
import math
import minimalmodbus 

from rclpy.callback_groups import ReentrantCallbackGroup
from test_move_xyz_theta_noflip_gmove1 import UR12eController

class PickAndPlaceBrain(UR12eController):
    def __init__(self):
        super().__init__()
        self.group = ReentrantCallbackGroup()
        self.pose_buffer = collections.deque(maxlen=10)
        self.is_busy = False
        self.current_class = ""
        self.target_class = "unknown"
        self.latest_pose = None
        self.latest_jaw_axis = None # NEW: Store jaw axis vector
        self.gripper_connected = False

        self.GRIPPER_OPEN = 3
        self.GRIPPER_CLOSED = 228 

        # Gripper Initialization
        print("Connecting to Gripper...")
        for i in range(3):
            try:
                self.gripper = minimalmodbus.Instrument('/tmp/ttyUR', 9)
                self.gripper.serial.baudrate = 115200
                self.gripper.serial.timeout = 0.1
                self.gripper.read_register(2000)
                self.gripper.write_register(1000, 0x0100)
                self.gripper_connected = True
                self.get_logger().info("Gripper Online.")
                break
            except Exception:
                time.sleep(0.5)

        # Subscribers
        self.obj_pose_sub = self.create_subscription(
            PoseStamped, '/grasp/pose', self.pose_cb, 10, callback_group=self.group)
        self.obj_class_sub = self.create_subscription(
            String, '/grasp/class', self.class_cb, 10, callback_group=self.group)
        
        # NEW: Subscription to jaw_axis from GraspNode4
        self.jaw_axis_sub = self.create_subscription(
            Vector3Stamped, '/grasp/jaw_axis', self.jaw_cb, 10, callback_group=self.group)

    def pose_cb(self, msg):
        if not self.is_busy:
            self.latest_pose = msg.pose

    def class_cb(self, msg):
        if not self.is_busy:
            self.current_class = msg.data
            self.target_class = msg.data

    def jaw_cb(self, msg):
        """NEW: Callback to handle jaw axis vector data"""
        if not self.is_busy:
            self.latest_jaw_axis = msg.vector

    def gripper_heartbeat(self):
        if not self.gripper_connected: return
        try:
            self.gripper.read_register(2000)
        except:
            pass

    def active_wait(self, seconds):
        start = time.time()
        while (time.time() - start) < seconds:
            self.gripper_heartbeat()
            time.sleep(0.05)

    def set_gripper(self, position):
        if not self.gripper_connected: return
        try:
            self.gripper.write_registers(1000, [0x0900, position, 0x6432])
        except:
            pass

    def get_theta_from_jaw_axis(self):
        """NEW: Calculates rotation angle based on the jaw axis vector"""
        if self.latest_jaw_axis is None:
            return 0.0 # Default if no jaw data yet
        
        # Calculate angle from X and Y vector components
        raw_theta = math.atan2(self.latest_jaw_axis.y, self.latest_jaw_axis.x)
        
        # Apply -90 degree offset so gripper fingers are perpendicular to axis
        return raw_theta - (math.pi / 2.0)

    def check_stability(self):
        if self.latest_pose is None or self.latest_pose.position.x == 0.0:
            return False
        if self.latest_jaw_axis is None: # Ensure jaw data is present
            return False
            
        self.pose_buffer.append([self.latest_pose.position.x, self.latest_pose.position.y])
        if len(self.pose_buffer) < 10: return False
        arr = np.array(self.pose_buffer)
        spread = np.max(arr, axis=0) - np.min(arr, axis=0)
        return all(spread < 0.02)

    def execute_move(self):
        self.is_busy = True
        p = self.latest_pose.position
        
        # UPDATED: Use jaw axis for theta calculation
        theta = -self.get_theta_from_jaw_axis()
        
        x_t, y_t = -0.03 - p.x, 1.28 - p.y
        z_t = max(0.242, 0.20 - 0.02 + p.z)

        self.get_logger().info(f"PICKING {self.target_class} with Jaw Alignment...")
        
        # Math Constants for Binning
        PI = math.pi
        D2R = PI / 180.0
        bin_hard = np.array([330.0, -65.72, -140.5, -63.9, 90.0, 60.0]) * D2R
        bin_soft = np.array([270.0, -54.27, -145.26, -70.48, 90.0, 0.0]) * D2R     
        
        try:
            # 1. Approach with jaw alignment
            self.set_gripper(self.GRIPPER_OPEN)
            self.move_xyz_theta_no_flip(x_t, y_t, z_t + 0.10, theta)
            self.active_wait(0.8)
            
            # 2. Pick
            self.move_xyz_theta_no_flip(x_t, y_t, z_t, theta)
            self.active_wait(0.7) 
            self.set_gripper(self.GRIPPER_CLOSED)
            self.active_wait(1.0)
            
            # 3. Lift and Sort
            self.move_xyz_theta_no_flip(x_t, y_t, z_t + 0.15, theta)
            self.active_wait(0.5)
            
            if self.current_class == "glove":
                self.jmove(bin_soft)
            else:
                self.jmove(bin_hard)    
                
            # 4. Release and Reset
            self.set_gripper(self.GRIPPER_OPEN)
            self.active_wait(0.5)
            self.move_xyz_theta_no_flip(0.0, 0.6, 0.5, 0.0)
            self.active_wait(0.8)
            
        except Exception as e:
            self.get_logger().error(f"Failed: {e}")
        finally:
            print("Cycle Finished.")
            os._exit(0)

def main(args=None):
    rclpy.init(args=args)
    brain = PickAndPlaceBrain()
    
    if brain.gripper_connected:
        print("Fast Activation...")
        brain.active_wait(2.5)

    print("READY. Waiting for Jaw-Aligned Target...")
    while rclpy.ok():
        rclpy.spin_once(brain, timeout_sec=0.01)
        brain.gripper_heartbeat()
        if brain.check_stability():
            brain.execute_move()

if __name__ == '__main__':
    main()