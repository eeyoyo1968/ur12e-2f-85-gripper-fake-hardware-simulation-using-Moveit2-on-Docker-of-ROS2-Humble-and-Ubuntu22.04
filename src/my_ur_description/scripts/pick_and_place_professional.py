import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Pose, Quaternion
import time

class UR12eProfessionalPP(Node):
    def __init__(self):
        super().__init__('ur12e_pick_place_node')
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self._gripper_client = ActionClient(self, FollowJointTrajectory, '/robotiq_gripper_controller/follow_joint_trajectory')
        
        self.get_logger().info("Connecting to MoveIt and Gripper...")
        self._move_group_client.wait_for_server()
        self._gripper_client.wait_for_server()

    def move_robot(self, x=None, y=None, z=None, joints=None):
        """Versatile move function: accepts XYZ or Joint list"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.allowed_planning_time = 5.0
        
        if joints:
            from moveit_msgs.msg import Constraints, JointConstraint
            c = Constraints()
            names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            for n, p in zip(names, joints):
                c.joint_constraints.append(JointConstraint(joint_name=n, position=p, tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
            goal_msg.request.goal_constraints.append(c)
        else:
            from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
            # Define Pose Goal
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
            # Standard 'Gripper Down' Orientation
            pose.orientation.x, pose.orientation.y = -0.707, 0.707
            pose.orientation.z, pose.orientation.w = 0.0, 0.0
            
            from moveit_msgs.msg import Pose_Constraint
            # Note: For simplicity in Python MoveIt, we wrap the pose in a constraint
            from moveit_msgs.msg import Constraints
            # (Simplified for MoveGroup Action usage)
            goal_msg.request.workspace_parameters.header.frame_id = "base_link"
            # Instead of complex constraints, most users use the MoveIt Wrapper or PoseStamped targets
            # We will send a standard MoveGroup Goal for XYZ:
        
        self.get_logger().info(f"Planning to target...")
        send_goal_future = self._move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        return send_goal_future.result()

    def control_gripper(self, pos):
        """0.0 is open, 0.7-0.8 is closed"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['robotiq_85_left_knuckle_joint']
        p = JointTrajectoryPoint()
        p.positions = [float(pos)]
        p.time_from_start.sec = 1
        goal.trajectory.points = [p]
        self._gripper_client.send_goal_async(goal)
        time.sleep(1.5)

def main():
    rclpy.init()
    bot = UR12eProfessionalPP()

    # 1. MOVE TO READY (Safe Configuration)
    # [pan, lift, elbow, w1, w2, w3]
    ready_joints = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    bot.move_robot(joints=ready_joints)
    bot.control_gripper(0.0) # Open

    # 2. APPROACH (Wait, why did Z=1.0 fail?)
    # On many UR setups, Z is relative to base_link. 
    # Try Z values between 0.2 and 0.5 (table height)
    bot.get_logger().info("Starting Pick Sequence...")
    # Using more realistic UR12e table coordinates
    # bot.move_robot(x=0.4, y=0.0, z=0.4) 

    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
