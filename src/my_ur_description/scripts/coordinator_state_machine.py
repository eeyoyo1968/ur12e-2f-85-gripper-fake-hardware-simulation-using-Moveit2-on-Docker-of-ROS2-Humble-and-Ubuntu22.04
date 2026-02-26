import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import collections
import numpy as np

from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from test_move_xyz_theta_noflip_gmove1 import UR12eController

class PickAndPlaceBrain(UR12eController):
    def __init__(self):
        super().__init__()
        # Use a reentrant group to allow the timer and subs to coexist
        self.group = ReentrantCallbackGroup()
        
        self.pose_buffer = collections.deque(maxlen=10) 
        self.is_busy = False
        self.latest_msg = None
        self.current_class = ""

        # Subscribers: We keep them alive but IGNORE them when busy
        self.obj_pose_sub = self.create_subscription(
            PoseStamped, '/grasp/pose', self.pose_cb, 10, callback_group=self.group)
        self.obj_class_sub = self.create_subscription(
            String, '/grasp/class', self.class_cb, 10, callback_group=self.group)

        # Heartbeat timer: This is the ONLY thing that triggers movement
        self.timer = self.create_timer(0.2, self.brain_logic, callback_group=self.group)

    def class_cb(self, msg):
        if not self.is_busy:
            self.current_class = msg.data

    def pose_cb(self, msg):
        if not self.is_busy:
            self.latest_msg = msg

    def brain_logic(self):
        # If the robot is moving, do absolutely nothing else
        if self.is_busy or self.latest_msg is None or self.current_class == "":
            return

        p = self.latest_msg.pose.position
        self.pose_buffer.append([p.x, p.y])
        
        if len(self.pose_buffer) >= 10:
            arr = np.array(self.pose_buffer)
            spread = np.max(arr, axis=0) - np.min(arr, axis=0)
            
            if all(spread < 0.02):
                self.get_logger().info(f"STABILITY MET! Target: {self.current_class}")
                # LOCK everything
                self.is_busy = True
                
                # Execute move SYNCHRONOUSLY (No threading!)
                self.execute_sequence(self.latest_msg.pose)

    def execute_sequence(self, pose):
        try:
            x_t = -0.03 - pose.position.x
            y_t = 1.28 - pose.position.y
            z_t = max(0.2275, 0.20 - 0.02 + pose.position.z)

            self.get_logger().info("--- STARTING PICK-PLACE SEQUENCE ---")
            
            # Step-by-step blocking moves
            self.move_xyz_no_flip(x_t, y_t, z_t + 0.15)
            self.move_xyz_no_flip(x_t, y_t, z_t)
            time.sleep(1.0) 
            self.move_xyz_no_flip(x_t, y_t, z_t + 0.2) 
        
            target_x = 0.2 if self.current_class == "glove" else -0.2
            self.move_xyz_no_flip(target_x, 0.4, 0.3)
            time.sleep(1.0)

            self.move_xyz_no_flip(0.0, 0.5, 0.5) 
            self.get_logger().info("--- SEQUENCE COMPLETE ---")

        except Exception as e:
            self.get_logger().error(f"Action failed: {e}")
        
        finally:
            # RESET everything for the next loop
            self.get_logger().info("Cooling down for 4 seconds...")
            self.pose_buffer.clear()
            self.latest_msg = None
            self.current_class = ""
            
            # This sleep allows the camera to see the empty table
            time.sleep(4.0) 
            self.is_busy = False
            self.get_logger().info("READY FOR NEXT OBJECT.")

def main(args=None):
    rclpy.init(args=args)
    brain = PickAndPlaceBrain()
    # Use SingleThreaded to force sequential execution
    executor = SingleThreadedExecutor()
    executor.add_node(brain)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Don't destroy subs manually, let rclpy handle it
        rclpy.shutdown()

if __name__ == '__main__':
    main()