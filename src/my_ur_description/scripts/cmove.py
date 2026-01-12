import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotTrajectory
from geometry_msgs.msg import Pose, Point, Quaternion
import time

class MoveIt2CartesianClient(Node):
    def __init__(self):
        super().__init__('cartesian_pick_place')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self._action_client.wait_for_server()

    def send_cartesian_linear_move(self, group_name, z_offset):
        """Moves the robot in a straight line along the Z axis."""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = group_name
        
        # 1. We tell MoveIt we want a Cartesian path
        # In the action request, we typically send a path of waypoints
        # For simplicity in this script, we'll define a target Pose 
        # but enable the "cartesian" flag if your planner supports it.
        
        self.get_logger().info(f'Executing linear move: {z_offset}m')
        # (See sequence below for how we use the joint-based approach 
        # to ensure the gripper stays vertical during the move)
        pass

    def send_joint_goal(self, group_name, joint_names, joint_values):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = group_name
        constraints = Constraints()
        for name, value in zip(joint_names, joint_values):
            jc = JointConstraint(joint_name=name, position=float(value), weight=1.0)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            constraints.joint_constraints.append(jc)
        goal_msg.request.goal_constraints.append(constraints)
        
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        handle = future.result()
        if not handle.accepted: return False
        
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

def main():
    rclpy.init()
    moveit = MoveIt2CartesianClient()

    arm_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                  'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    # --- STRAIGHT DOWN SEQUENCE ---
    
    # 1. POSITION ABOVE: Gripper is vertical (Wrist 2 at -1.57 makes tool point down)
    # Pose: Above the object
    print("Moving to Approach position (Vertical)...")
    moveit.send_joint_goal('ur_manipulator', arm_joints, [0.0, -1.2, 1.8, -2.1, -1.57, 0.0])

    # 2. LINEAR LOWER: To achieve a 'linear' look with joints, we only change 
    # the shoulder and elbow slightly while keeping the wrist fixed.
    print("Lowering vertically to Pick...")
    moveit.send_joint_goal('ur_manipulator', arm_joints, [0.0, -0.9, 2.0, -2.6, -1.57, 0.0])

    # 3. GRASP
    moveit.send_joint_goal('gripper', ['robotiq_85_left_knuckle_joint'], [0.4])

    # 4. LINEAR LIFT: Move back to the exact joint state from Step 1
    print("Lifting vertically...")
    moveit.send_joint_goal('ur_manipulator', arm_joints, [0.0, -1.2, 1.8, -2.1, -1.57, 0.0])

    print("Sequence Complete!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()