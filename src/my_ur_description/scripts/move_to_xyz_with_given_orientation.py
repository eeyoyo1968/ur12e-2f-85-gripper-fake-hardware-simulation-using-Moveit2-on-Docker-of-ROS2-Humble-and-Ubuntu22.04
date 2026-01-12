import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

class UR12eNaturalMove(Node):
    def __init__(self):
        super().__init__('ur12e_natural_move')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self._ik_client = self.create_client(GetPositionIK, 'compute_ik')
        
        # Subscribe to current joints to use as a seed
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.current_state = RobotState()
        
        self.get_logger().info('Waiting for MoveGroup and IK Service...')
        self._action_client.wait_for_server()
        self._ik_client.wait_for_service()

        # Your captured "reasonable" orientation
        self.qx, self.qy, self.qz, self.qw = -0.707, 0.707, 0.001, 0.000

    def joint_cb(self, msg):
        self.current_state.joint_state = msg

    def move_to_xyz_natural(self, x, y, z):
        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = "ur_manipulator"
        ik_req.ik_request.avoid_collisions = True
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.pose.position.x = float(x)
        target_pose.pose.position.y = float(y)
        target_pose.pose.position.z = float(z)
        target_pose.pose.orientation.x = self.qx
        target_pose.pose.orientation.y = self.qy
        target_pose.pose.orientation.z = self.qz
        target_pose.pose.orientation.w = self.qw
        ik_req.ik_request.pose_stamped = target_pose

        # --- TRY 1: NATURAL SEEDED MOVE ---
        ik_req.ik_request.robot_state = self.current_state
        self.get_logger().info("Attempting Seeded IK...")
        
        future = self._ik_client.call_async(ik_req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        if res.error_code.val == 1:
            self.get_logger().info("Seeded IK Found!")
            return self.send_joint_goal(res.solution.joint_state)
        
        # --- TRY 2: RELAXED UNSEEDED MOVE (FALLBACK) ---
        self.get_logger().warn("Seeded IK failed (-31). Retrying with Relaxed IK...")
        ik_req.ik_request.robot_state = RobotState() # Clear the seed
        
        future = self._ik_client.call_async(ik_req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        if res.error_code.val == 1:
            self.get_logger().info("Relaxed IK Found! (Note: This might result in a larger movement)")
            return self.send_joint_goal(res.solution.joint_state)
        else:
            self.get_logger().error(f"Point unreachable: IK Error {res.error_code.val}")
            return False

    def send_joint_goal(self, joint_state):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.max_velocity_scaling_factor = 0.2
        
        c = Constraints()
        for name, pos in zip(joint_state.name, joint_state.position):
            if "robotiq" not in name: # Only move the arm joints
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = pos
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                c.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(c)
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        return True

def main():
    rclpy.init()
    bot = UR12eNaturalMove()
    # Let it get the first joint state
    for _ in range(10): rclpy.spin_once(bot, timeout_sec=0.1)
    
    bot.move_to_xyz_natural(0.7, 0.0, 1.5)
    bot.move_to_xyz_natural(0.7, 0.0, 1.2)
    bot.move_to_xyz_natural(0.7, 0.0, 1.5)
    bot.move_to_xyz_natural(0.7, 0.1, 1.5)
    bot.move_to_xyz_natural(0.7, 0.1, 1.2)
    bot.move_to_xyz_natural(0.7, 0.1, 1.5)
    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()