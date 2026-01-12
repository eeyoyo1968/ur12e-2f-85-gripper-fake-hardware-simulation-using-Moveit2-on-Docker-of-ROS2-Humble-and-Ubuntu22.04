import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
import time

class MoveIt2Client(Node):
    def __init__(self):
        super().__init__('pick_and_place_client')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('Waiting for MoveGroup action server...')
        self._action_client.wait_for_server()

    def send_joint_goal(self, group_name, joint_names, joint_values):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = group_name
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        constraints = Constraints()
        for name, value in zip(joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(constraints)
        
        self.get_logger().info(f'Planning for {group_name}...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False

        self.get_logger().info('Goal accepted, executing...')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        return True

def main():
    rclpy.init()
    moveit = MoveIt2Client()

    # Joint Name Definitions
    arm_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                  'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    gripper_joints = ['robotiq_85_left_knuckle_joint']

    # --- THE SEQUENCE ---

    # 1. Open Gripper
    print("\n[Step 1/6] Opening Gripper...")
    moveit.send_joint_goal('gripper', gripper_joints, [0.0])

    # 2. Move to Pre-Pick (Above the object)
    print("\n[Step 2/6] Moving to Pre-Pick...")
    moveit.send_joint_goal('ur_manipulator', arm_joints, [0.0, -1.2, 1.8, -2.1, -1.57, 0.0])

    # 3. Move to Pick (Down to object)
    print("\n[Step 3/6] Lowering to Pick...")
    moveit.send_joint_goal('ur_manipulator', arm_joints, [0.0, -0.9, 2.0, -2.6, -1.57, 0.0])

    # 4. Close Gripper (Grasp)
    print("\n[Step 4/6] Grasping...")
    moveit.send_joint_goal('gripper', gripper_joints, [0.6]) # 0.8 is full close
    time.sleep(1.5)

    # 5. Lift (Move back up)
    print("\n[Step 5/6] Lifting...")
    moveit.send_joint_goal('ur_manipulator', arm_joints, [0.0, -1.2, 1.8, -2.1, -1.57, 0.0])

    # 6. Move to Place Position (Example: rotate base)
    print("\n[Step 6/6] Moving to Place...")
    moveit.send_joint_goal('ur_manipulator', arm_joints, [0.25, -1.2, 1.8, -2.1, -1.57, 0.0])
    
    # Release
    moveit.send_joint_goal('gripper', gripper_joints, [0.0])

    print("\n--- Sequence Complete! ---")
    moveit.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()