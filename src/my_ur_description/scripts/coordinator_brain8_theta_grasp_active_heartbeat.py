import rclpy
from geometry_msgs.msg import PoseStamped
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
        self.gripper_connected = False

        self.GRIPPER_OPEN = 3
        self.GRIPPER_CLOSED = 228 

        print("Connecting to Gripper on /tmp/ttyUR...")
        for i in range(5):
            try:
                self.gripper = minimalmodbus.Instrument('/tmp/ttyUR', 9)
                self.gripper.serial.baudrate = 115200
                self.gripper.serial.timeout = 0.2
                self.gripper.read_register(2000)
                self.gripper.write_register(1000, 0x0100)
                self.gripper_connected = True
                self.get_logger().info("Gripper Online & Activating.")
                break
            except Exception:
                self.get_logger().warn(f"Gripper not responding (Attempt {i+1}/5)...")
                time.sleep(1.0)

        self.obj_pose_sub = self.create_subscription(
            PoseStamped, '/grasp/pose', self.pose_cb, 10, callback_group=self.group)
        self.obj_class_sub = self.create_subscription(
            String, '/grasp/class', self.class_cb, 10, callback_group=self.group)

    def pose_cb(self, msg):
        if not self.is_busy:
            self.latest_pose = msg.pose

    def class_cb(self, msg):
        if not self.is_busy:
            self.current_class = msg.data
            self.target_class = msg.data

    def set_gripper(self, position):
        if not self.gripper_connected: return
        try:
            self.gripper.write_registers(1000, [0x0900, position, 0x6432])
        except:
            pass

    def gripper_heartbeat(self):
        """Pings the gripper to prevent Red LED watchdog timeout"""
        if not self.gripper_connected: return
        try:
            self.gripper.read_register(2000)
        except:
            pass

    def active_wait(self, seconds):
        """Wait while keeping the gripper alive every 100ms"""
        start = time.time()
        while (time.time() - start) < seconds:
            self.gripper_heartbeat()
            time.sleep(0.1)

    def get_theta_from_pose(self, pose):
        q = pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def check_stability(self):
        if self.latest_pose is None or self.latest_pose.position.x == 0.0:
            return False
        self.pose_buffer.append([self.latest_pose.position.x, self.latest_pose.position.y])
        if len(self.pose_buffer) < 10: return False
        arr = np.array(self.pose_buffer)
        spread = np.max(arr, axis=0) - np.min(arr, axis=0)
        return all(spread < 0.02)

    def execute_move(self):
        self.is_busy = True
        p = self.latest_pose.position
        theta = self.get_theta_from_pose(self.latest_pose)
        
        # Optimized Z-height for faster descent
        x_t, y_t = -0.03 - p.x, 1.28 - p.y
        z_t = max(0.232, 0.20 - 0.02 + p.z)

        self.get_logger().info(f"PICKING {self.target_class}...")
        
        try:
            # 1. Open and Approach
            self.set_gripper(self.GRIPPER_OPEN)
            self.move_xyz_theta_no_flip(x_t, y_t, z_t + 0.12, theta)
            self.active_wait(1.2) # Snappier wait
            
            # 2. Descend
            self.move_xyz_theta_no_flip(x_t, y_t, z_t, theta)
            self.active_wait(1.0) 
            
            # 3. Grasp
            self.set_gripper(self.GRIPPER_CLOSED)
            self.active_wait(1.2) # Firm grasp time
            
            # 4. Lift and Bin
            self.move_xyz_theta_no_flip(x_t, y_t, z_t + 0.15, theta)
            self.active_wait(0.8)
            
            bin_x = 0.25 if self.current_class == "glove" else -0.25
            self.move_xyz_theta_no_flip(bin_x, 0.45, 0.35, 0.0)
            self.active_wait(1.8) # Travel time
            
            # 5. Release and Fast Return
            self.set_gripper(self.GRIPPER_OPEN)
            self.active_wait(0.6)
            self.move_xyz_theta_no_flip(0.0, 0.6, 0.5, 0.0)
            self.active_wait(1.0)
            
        except Exception as e:
            self.get_logger().error(f"Failed: {e}")
        finally:
            print("Cycle Finished.")
            os._exit(0)

def main(args=None):
    rclpy.init(args=args)
    brain = PickAndPlaceBrain()
    
    if brain.gripper_connected:
        print("Waiting 4s for Activation...")
        time.sleep(4.0)

    print("READY. Place object...")
    while rclpy.ok():
        rclpy.spin_once(brain, timeout_sec=0.05)
        brain.gripper_heartbeat() # Keep Blue while idle
        
        if brain.check_stability():
            brain.execute_move()

if __name__ == '__main__':
    main()