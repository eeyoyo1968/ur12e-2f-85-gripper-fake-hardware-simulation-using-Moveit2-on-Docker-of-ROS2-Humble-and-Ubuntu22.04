import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
import time

class UR12eMasterController(Node):
    def __init__(self):
        super().__init__('ur12e_master_controller')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.attach_pub = self.create_publisher(AttachedCollisionObject, '/attached_collision_object', 10)
        self.scene_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        
        self.get_logger().info('Master Controller Initialized. Waiting for MoveGroup...')
        self._action_client.wait_for_server()

    def send_joint_goal(self, group_name, joints, values, speed=0.1):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = group_name
        goal_msg.request.max_velocity_scaling_factor = speed
        
        # INCREASE PLANNING TOLERANCE
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0  # Give MoveIt more time to find a valid path
        
        c = Constraints()
        for name, val in zip(joints, values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(val)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        goal_msg.request.goal_constraints.append(c)
        
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        handle = future.result()
        if handle:
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
        return True

    def manage_attachment(self, object_id, action="attach"):
        att = AttachedCollisionObject()
        att.link_name = "robotiq_85_left_finger_tip_link"
        att.object.id = object_id
        att.object.header.frame_id = att.link_name
        
        # List all gripper parts that are allowed to touch the box
        att.touch_links = [
            "robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link",
            "robotiq_85_left_inner_finger_pad_link", "robotiq_85_right_inner_finger_pad_link",
            "robotiq_85_left_inner_finger", "robotiq_85_right_inner_finger",
            "target_box" # Allow the robot to touch the object itself
        ]
        
        if action == "attach":
            att.object.operation = b'\x00' 
        else:
            att.object.operation = b'\x01' 
            
        self.attach_pub.publish(att)
        self.get_logger().info(f"Object {object_id} {action}ed!")

def main():
    rclpy.init()
    bot = UR12eMasterController()
    
    arm_j = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    grip_j = ['robotiq_85_left_knuckle_joint']

    try:
        while rclpy.ok():
            # 1. PRE-PICK (Safe height)
            bot.send_joint_goal('gripper', grip_j, [0.0], speed=0.5)
            bot.send_joint_goal('ur_manipulator', arm_j, [0.0, -1.0, 1.8, -2.3, -1.57, 0.0], speed=0.2)

            # 2. PICK (Higher than before to avoid wrist/desk collision)
            # Lift: -0.9, Elbow: 1.9, Wrist1: -2.5
            bot.send_joint_goal('ur_manipulator', arm_j, [0.0, -0.9, 1.9, -2.5, -1.57, 0.0], speed=0.1)

            # 3. ATTACH
            bot.send_joint_goal('gripper', grip_j, [0.4], speed=0.2)
            bot.manage_attachment("target_box", "attach")
            time.sleep(1.0) # Wait for attachment to register

            # 4. LIFT
            bot.send_joint_goal('ur_manipulator', arm_j, [0.0, -1.1, 1.9, -2.4, -1.57, 0.0], speed=0.2)

            # 5. MOVE TO PLACE (Reduced rotation to 0.5 to avoid left wall collision)
            bot.send_joint_goal('ur_manipulator', arm_j, [0.5, -1.1, 1.9, -2.4, -1.57, 0.0], speed=0.2)

            # 6. LOWER TO PLACE (Same height as pick)
            bot.send_joint_goal('ur_manipulator', arm_j, [0.5, -0.9, 1.9, -2.5, -1.57, 0.0], speed=0.1)
            
            # 7. RELEASE & DETACH
            bot.manage_attachment("target_box", "detach")
            bot.send_joint_goal('gripper', grip_j, [0.0], speed=0.3)

            # 8. RETRACT
            bot.send_joint_goal('ur_manipulator', arm_j, [0.5, -1.1, 1.9, -2.4, -1.57, 0.0], speed=0.2)
            print("Cycle complete!")

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        if rclpy.ok():
            bot.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()