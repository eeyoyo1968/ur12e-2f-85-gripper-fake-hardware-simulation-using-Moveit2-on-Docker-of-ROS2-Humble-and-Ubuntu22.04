import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import math
import numpy as np

# Messages
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState 
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from moveit_msgs.msg import PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest

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
        self.get_logger().info("System Online. Controllers Active.")

    def joint_state_callback(self, msg):
        joint_name = 'robotiq_85_left_knuckle_joint'
        if joint_name in msg.name:
            idx = msg.name.index(joint_name)
            self.current_gripper_pos = msg.position[idx]

    def gripper_move(self, pos):
        """Standard Action-based gripper control"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['robotiq_85_left_knuckle_joint']
        point = JointTrajectoryPoint()
        point.positions = [float(pos)]
        point.time_from_start.sec = 2
        goal_msg.trajectory.points.append(point)
        
        self.get_logger().info(f"Gripper -> {pos}")
        future = self._gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return True

    def jmove(self, jointvector):
        """Joint-space motion"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.start_state.is_diff = True 
        
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
        return True

    def move_pose_no_flip(self, x, y, z, ox, oy, oz, ow, frame_id="base_link"):
        """Cartesian move with IK Seeding to prevent joint flipping"""
        ik_client = self.create_client(GetPositionIK, 'compute_ik')
        ik_client.wait_for_service()

        request = GetPositionIK.Request()
        ik_req = PositionIKRequest()
        ik_req.group_name = "ur_manipulator"
        ik_req.avoid_collisions = True
        
        # Target
        target = PoseStamped()
        target.header.frame_id = frame_id
        target.pose.position = Point(x=float(x), y=float(y), z=float(z))
        target.pose.orientation = Quaternion(x=float(ox), y=float(oy), z=float(oz), w=float(ow))
        ik_req.pose_stamped = target

        # Seed (Home configuration to avoid flips)
        home_seed = [-1.5707, -2.3562, 2.3562, -1.5707, -1.5707, 0.0]
        ik_req.robot_state.joint_state.name = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        ik_req.robot_state.joint_state.position = home_seed
        
        request.ik_request = ik_req
        future = ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()

        if resp.error_code.val == 1:
            return self.jmove(list(resp.solution.joint_state.position[:6]))
        else:
            self.get_logger().error(f"IK Fail: {resp.error_code.val}")
            return False

def main():
    rclpy.init()
    bot = UR12eController()
    home = [-1.5707, -2.3562, 2.3562, -1.5707, -1.5707, 0.0]

    # sequence
    bot.jmove(home)
    # Pick approach (Top-down orientation: x=1, y=0, z=0, w=0)
    bot.move_pose_no_flip(0.6, -0.2, 0.5, 1.0, 0.0, 0.0, 0.0)
    bot.gripper_move(0.0)
    
    # Lower and Grab
    bot.move_pose_no_flip(0.6, -0.2, 0.3, 1.0, 0.0, 0.0, 0.0)
    bot.gripper_move(0.8)
    
    # Move to Place
    bot.move_pose_no_flip(0.6, -0.2, 0.5, 1.0, 0.0, 0.0, 0.0)
    bot.move_pose_no_flip(0.6, 0.2, 0.5, 1.0, 0.0, 0.0, 0.0)
    bot.move_pose_no_flip(0.6, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0)
    bot.gripper_move(0.0)
    
    bot.jmove(home)
    bot.get_logger().info("Demo Complete.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()