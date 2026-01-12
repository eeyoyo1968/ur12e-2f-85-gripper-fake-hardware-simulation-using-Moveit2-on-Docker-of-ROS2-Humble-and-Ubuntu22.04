import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes

class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

    def move_to_joints(self, group_name, joint_names, joint_values):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = group_name
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        constraints = Constraints()
        for name, value in zip(joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(constraints)
        
        self.get_logger().info(f'Sending goal to {group_name}...')
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    node = PickAndPlace()

    # Joint Names for UR12e
    arm_joints = [
        'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
        'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
    ]
    
    # 1. Move to Home/Pick position (Radiant values)
    # [pan, lift, elbow, w1, w2, w3]
    node.move_to_joints('ur_manipulator', arm_joints, [0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
    
    print("Execution triggered. Check RViz for movement!")
    
    # Keep node spinning to process the action
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()