import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import math
import numpy as np

# Messages
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState 

class UR12eController(Node):
    def __init__(self):
        super().__init__('ur12e_controller')
        
        # Action Clients
        self._arm_client = ActionClient(self, MoveGroup, 'move_action')
        self._gripper_client = ActionClient(self, FollowJointTrajectory, '/robotiq_gripper_controller/follow_joint_trajectory')

        # Feedback variables
        self.current_gripper_pos = 0.0
        
        # Subscriber to Joint States
        self._joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        self.get_logger().info("Connecting to controllers...")
        self._arm_client.wait_for_server()
        self._gripper_client.wait_for_server()
        self.get_logger().info("System Online.")

    def joint_state_callback(self, msg):
        """Updates the gripper knuckle joint position in real-time."""
        joint_name = 'robotiq_85_left_knuckle_joint'
        if joint_name in msg.name:
            idx = msg.name.index(joint_name)
            self.current_gripper_pos = msg.position[idx]

    def gripper_move(self, pos):
        """Moves the gripper and waits for the action to finish."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['robotiq_85_left_knuckle_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [float(pos)]
        point.time_from_start.sec = 1
        goal_msg.trajectory.points.append(point)
        
        future = self._gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        handle = future.result()
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        
        time.sleep(0.8) # Allow simulation physics to stabilize
        return True

    def check_grasp_success(self):
        """
        Logic: If position > 0.75, the gripper is empty.
        In simulation, this is highly accurate.
        """
        self.get_logger().info(f"Grasp Check - Position: {self.current_gripper_pos:.4f}")
        return self.current_gripper_pos < 0.75

    def jmove(self, jointvector):
        # ... [Your existing jmove code here] ...
        pass

def main():
    rclpy.init()
    bot = UR12eController()
    
    # Constants
    D2R = math.pi / 180.0
    home = [-1.5707, -2.3562, 2.3562, -1.5707, -1.5707, 0.0]
    pick_depth = np.array([-16.0, -72.0, 113.0, -129.0, -90.0, 86.0]) * D2R

    max_retries = 3

    try:
        # Move to initial home
        bot.jmove(home)

        # START PICK ATTEMPT
        success = False
        attempt = 0

        while attempt < max_retries and not success:
            attempt += 1
            bot.get_logger().info(f"Pick Attempt {attempt} of {max_retries}")
            
            # 1. Open and Approach
            bot.gripper_move(0.0) 
            bot.jmove(pick_depth)

            # 2. Command Full Close
            bot.gripper_move(0.8) 

            # 3. Check Grasp
            if bot.check_grasp_success():
                bot.get_logger().info("Grasp SUCCESSFUL!")
                success = True
            else:
                bot.get_logger().warn("Grasp FAILED: Empty.")
                bot.gripper_move(0.0) # Open back up
                # Optional: Add a small jitter/offset here to try a different spot
                time.sleep(1.0)

        if success:
            bot.get_logger().info("Proceeding to next task...")
            bot.jmove(home)
            bot.gripper_move(0.0)
        else:
            bot.get_logger().error("Critical: Failed to pick object after multiple attempts.")
            bot.jmove(home)

    finally:
        bot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()