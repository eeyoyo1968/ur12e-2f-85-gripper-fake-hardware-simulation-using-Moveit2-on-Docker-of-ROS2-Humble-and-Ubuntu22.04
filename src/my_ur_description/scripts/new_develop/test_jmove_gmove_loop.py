import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import math
import numpy as np

# MoveIt and Control Messages
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class UR12eController(Node):
    def __init__(self):
        super().__init__('ur12e_controller')
        
        # Action Clients
        self._arm_client = ActionClient(self, MoveGroup, 'move_action')
        self._gripper_client = ActionClient(self, FollowJointTrajectory, '/robotiq_gripper_controller/follow_joint_trajectory')

        self.get_logger().info("Connecting to controllers...")
        self._arm_client.wait_for_server()
        self._gripper_client.wait_for_server()
        self.get_logger().info("System Online: Ready for loops.")

    def jmove(self, jointvector):
        """Moves the UR12e Arm to a joint configuration"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.start_state.is_diff = True 
        goal_msg.request.allowed_planning_time = 5.0
        
        joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        con = Constraints()
        for name, pos in zip(joint_names, jointvector):
            jc = JointConstraint(joint_name=name, position=float(pos), 
                                 tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            con.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(con)
        future = self._arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted: return False
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return True

    def gripper_move(self, pos):
        """Controls the Robotiq Gripper (0.0=Open, 0.8=Closed)"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['robotiq_85_left_knuckle_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [float(pos)]
        point.time_from_start.sec = 1
        
        goal_msg.trajectory.points.append(point)
        future = self._gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        res_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return True

def main():
    rclpy.init()
    bot = UR12eController()

    # --- Math Constants ---
    PI = math.pi
    D2R = PI / 180.0

    # --- Pose Definitions ---
    # Home (Shoulder at -90 degrees)
    home = [-1.5707, -2.3562, 2.3562, -1.5707, -1.5707, 0.0]
    
    # Pick Positions (Calculated from your degree inputs)
    pick_pos = np.array([-16.0, -74.0, 113.0, -129.0, -90.0, 74.0]) * D2R
    pick_depth = np.array([-16.0, -72.0, 113.0, -129.0, -90.0, 86.0]) * D2R
    
    # Safe travel pose
    up_pose = [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0]

    loop_count = 0
    max_loops = 5  # Set how many times you want it to run

    try:
        while loop_count < max_loops:
            loop_count += 1
            bot.get_logger().info(f"=== Starting Cycle {loop_count} of {max_loops} ===")

            # 1. Open Gripper and go Home
            bot.gripper_move(0.0) 
            bot.jmove(home)

            # 2. Move to Pick Position
            bot.get_logger().info("Approaching target...")
            bot.jmove(pick_pos)
            bot.jmove(pick_depth)

            # 3. Close Gripper (Simulated Grasp)
            bot.get_logger().info("Grasping...")
            bot.gripper_move(0.6) # 0.8 is fully closed; 0.6 usually hits the object
            time.sleep(0.5)

            # 4. Lift and move to 'Up' pose
            bot.get_logger().info("Lifting...")
            bot.jmove(up_pose)

            # 5. Return Home and Release
            bot.get_logger().info("Returning home and releasing...")
            bot.jmove(home)
            bot.gripper_move(0.0)
            
            bot.get_logger().info(f"Cycle {loop_count} Complete.")
            time.sleep(1.0) # Brief pause between cycles

    except KeyboardInterrupt:
        bot.get_logger().info("Loop interrupted by user.")
    finally:
        bot.get_logger().info("Shutting down controller.")
        bot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()