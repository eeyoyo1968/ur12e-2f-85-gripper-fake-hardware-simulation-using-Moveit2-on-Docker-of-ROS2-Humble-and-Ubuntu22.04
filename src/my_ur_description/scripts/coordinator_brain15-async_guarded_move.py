import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped  # Added WrenchStamped
from std_msgs.msg import String
import time
import collections
import numpy as np
import os 
import math
import minimalmodbus 
import threading
from rclpy.executors import MultiThreadedExecutor # Add this import


from rclpy.callback_groups import ReentrantCallbackGroup
from test_move_xyz_theta_noflip_gmove1 import UR12eController

class PickAndPlaceBrain(UR12eController):
    def __init__(self):
        super().__init__()
        self.serial_lock = threading.Lock() # NEW: Gripper lock
        self.group = ReentrantCallbackGroup()
        self.pose_buffer = collections.deque(maxlen=10)
        self.is_busy = False
        self.current_class = ""
        self.target_class = "unknown"
        self.latest_pose = None
        self.gripper_connected = False
        
        # NEW: Force-Sensing Variables
        self.current_force_z = 0.0
        #self.force_threshold = 7.0  # Newtons
        self.force_threshold = 2.0  # Newtons
        self.force_tare_z = 0.0

        self.GRIPPER_OPEN = 3
        self.GRIPPER_CLOSED = 228 

        # Gripper Init (from Brain 10)
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

        self.obj_pose_sub = self.create_subscription(
            PoseStamped, '/grasp/pose', self.pose_cb, 10, callback_group=self.group)
        self.obj_class_sub = self.create_subscription(
            String, '/grasp/class', self.class_cb, 10, callback_group=self.group)
        
        # NEW: Force sensor subscription
        self.wrench_sub = self.create_subscription(
            WrenchStamped, '/wrench', self.wrench_cb, 10, callback_group=self.group)

    def pose_cb(self, msg):
        if not self.is_busy:
            self.latest_pose = msg.pose

    def class_cb(self, msg):
        if not self.is_busy:
            self.current_class = msg.data
            self.target_class = msg.data

    def wrench_cb(self, msg):
        self.current_force_z = msg.wrench.force.z

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
        with self.serial_lock: # NEW: Wrap serial access in lock
            try:
                self.gripper.write_registers(1000, [0x0900, position, 0x6432])
            except:
                pass

    def get_theta_from_pose(self, pose):
        """Keeping your preferred quaternion-to-yaw logic"""
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
        theta = -self.get_theta_from_pose(self.latest_pose) + math.pi/2
        
        x_t, y_t = -0.03 - p.x, 1.28 - p.y
        z_target = max(0.233, 0.20 - 0.02 + p.z) 
        z_start_search = z_target + 0.015  # 15mm above table
        z_floor = z_target - 0.020         # Safety "floor"
        z_approach = z_target + 0.10

        # Binning targets
        PI = math.pi
        D2R = PI / 180.0
        bin_hard = np.array([330.0, -65.72, -140.5, -63.9, 90.0, 60.0]) * D2R
        bin_soft = np.array([270.0, -54.27, -145.26, -70.48, 90.0, 0.0]) * D2R   
        
        try:
            # 1. Approach and Tare
            self.set_gripper(self.GRIPPER_OPEN)
            self.move_xyz_theta_no_flip(x_t, y_t, z_start_search, theta)
            self.active_wait(0.5)
            self.force_tare_z = self.current_force_z

            if z_target <= 0.26:
                self.get_logger().info("SMOOTH SEARCH STARTING...")
                qx = math.cos(theta/2.0); qy = -math.sin(theta/2.0); qz=0.0; qw=0.0
                target_joints = self.get_ik_pose(x_t, y_t, z_floor, qx, qy, qz, qw)
                
                if target_joints:
                    move_future = self.jmove_async(target_joints) # NEW: Async move
                    
                    start_time = time.time()
                    while rclpy.ok() and (time.time() - start_time) < 5.0:
                        # DEBUG: Print raw values to see if they are changing
                        force_delta = abs(self.current_force_z - self.force_tare_z)
                        if int(time.time() * 10) % 5 == 0: # Print every 0.5 seconds
                            self.get_logger().info(f"Monitoring - Current: {self.current_force_z:.2f}, Tare: {self.force_tare_z:.2f}, Delta: {force_delta:.2f}")

                        if force_delta > self.force_threshold:
                            self.stop_motion()
                            self.get_logger().info(f"TOUCH! Force: {force_delta:.2f}N. Retracting...")
                            # RETRACT move as requested
                            self.move_xyz_theta_no_flip(x_t, y_t, z_target + 0.005, theta)
                            break
    
                        if move_future.done(): 
                            self.get_logger().info("Motion finished naturally without contact.")
                            break
    
                        self.gripper_heartbeat()
                        time.sleep(0.01)
                else:
                    self.get_logger().error("Could not find IK solution for search floor.")
            else:
                self.move_xyz_theta_no_flip(x_t, y_t, z_target, theta)

            # 2. GRASP
            self.active_wait(0.2)
            self.set_gripper(self.GRIPPER_CLOSED)
            self.active_wait(1.2) 
            
            # 3. Lift and Sort
            self.move_xyz_theta_no_flip(x_t, y_t, z_approach, theta)
            if self.current_class == "glove":
                self.jmove(bin_soft)
            else:
                self.jmove(bin_hard)    
                
            self.set_gripper(self.GRIPPER_OPEN)
            self.active_wait(2.0)
            self.move_xyz_theta_no_flip(0.0, 0.6, 0.5, 0.0)
            
        except Exception as e:
            self.get_logger().error(f"Failed: {e}")
        finally:
            os._exit(0)





# ... (Main loop remains identical to Brain 10)

def main(args=None):
    rclpy.init(args=args)
    brain = None
    try:
        brain = PickAndPlaceBrain()
        # Use MultiThreadedExecutor to allow concurrent callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(brain)
        
        if brain.gripper_connected:
            print("READY (Guarded Move Enabled).")

        # Start the executor in a background thread or use spin
        # To keep your existing logic, we will use a separate thread for the executor
        import threading
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        while rclpy.ok():
            # heartbeat can still run here
            brain.gripper_heartbeat()
            if brain.check_stability():
                brain.execute_move()
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        if brain:
            brain.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()