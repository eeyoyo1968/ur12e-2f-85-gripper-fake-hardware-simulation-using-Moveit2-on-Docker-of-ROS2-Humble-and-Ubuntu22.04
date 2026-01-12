import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
import time

class GracefulMoveClient(Node):
    def __init__(self):
        super().__init__('graceful_picker')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self._action_client.wait_for_server()

    def send_controlled_goal(self, group_name, joint_names, joint_values, speed=0.1):
        """
        Sends a joint goal with custom velocity and acceleration scaling.
        speed: float from 0.01 to 1.0
        """
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = group_name
        
        # --- SPEED CONTROL ---
        goal_msg.request.max_velocity_scaling_factor = float(speed)
        goal_msg.request.max_acceleration_scaling_factor = float(speed)
        
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        constraints = Constraints()
        for name, value in zip(joint_names, joint_values):
            jc = JointConstraint(joint_name=name, position=float(value), weight=1.0)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(constraints)
        
        self.get_logger().info(f'Moving {group_name} at {speed*100}% speed...')
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        handle = future.result()
        if not handle or not handle.accepted:
            return False
            
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return True

def main():
    rclpy.init()
    moveit = GracefulMoveClient()

    arm_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                  'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    # 1. Travel fast to the workspace
    moveit.send_controlled_goal('ur_manipulator', arm_joints, [0.0, -1.2, 1.8, -2.1, -1.57, 0.0], speed=0.5)

    # 2. Slow, graceful descent to pick
    print("Approaching object slowly...")
    moveit.send_controlled_goal('ur_manipulator', arm_joints, [0.0, -0.9, 2.0, -2.6, -1.57, 0.0], speed=0.05)

    # 3. Close Gripper
    moveit.send_controlled_goal('gripper', ['robotiq_85_left_knuckle_joint'], [0.4], speed=0.2)
    time.sleep(0.5)

    # 4. Slow lift to avoid shaking the "object"
    print("Lifting gracefully...")
    moveit.send_controlled_goal('ur_manipulator', arm_joints, [0.0, -1.2, 1.8, -2.1, -1.57, 0.0], speed=0.1)

    print("Sequence Complete!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()