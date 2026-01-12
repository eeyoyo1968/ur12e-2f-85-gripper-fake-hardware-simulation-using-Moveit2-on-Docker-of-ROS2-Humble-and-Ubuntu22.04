import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import time  # <--- Added this missing import

class UR12eStableMove(Node):
    def __init__(self):
        super().__init__('ur12e_stable_move')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self._ik_client = self.create_client(GetPositionIK, 'compute_ik')
        
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.current_state = RobotState()
        self.has_joints = False

        self.get_logger().info('Waiting for MoveGroup and IK Service...')
        self._action_client.wait_for_server()
        self._ik_client.wait_for_service()

        # Your exact orientation from TF
        self.qx, self.qy, self.qz, self.qw = -0.707, 0.707, 0.001, 0.000

    def joint_cb(self, msg):
        self.current_state.joint_state = msg
        self.has_joints = True

    def move_to_xyz(self, x, y, z):
        # Wait for the first joint state if it hasn't arrived yet
        while not self.has_joints:
            self.get_logger().warn("Waiting for joint states...")
            rclpy.spin_once(self, timeout_sec=0.1)

        # Prepare Request
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

        # --- STEP 1: Try seeded (Natural) IK ---
        ik_req.ik_request.robot_state = self.current_state
        self.get_logger().info(f"Attempting move to {x}, {y}, {z}...")
        
        future = self._ik_client.call_async(ik_req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        # --- STEP 2: Fallback to Unseeded (Global) IK if Seeded fails ---
        if res.error_code.val != 1:
            self.get_logger().warn("Seeded IK failed. Retrying with neutral seed...")
            ik_req.ik_request.robot_state = RobotState() # Clear the seed
            future = self._ik_client.call_async(ik_req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()

        if res.error_code.val == 1:
            return self.execute_joint_goal(res.solution.joint_state)
        else:
            self.get_logger().error(f"IK Failed completely. Error code: {res.error_code.val}")
            return False

    def execute_joint_goal(self, joint_state):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        # Slow speed for safety
        goal_msg.request.max_velocity_scaling_factor = 0.1
        
        c = Constraints()
        for name, pos in zip(joint_state.name, joint_state.position):
            if "robotiq" not in name:
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
        
        handle = future.result()
        if handle and handle.accepted:
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            return True
        return False

def main():
    rclpy.init()
    bot = UR12eStableMove()
    
    # Testing the sequence
    bot.move_to_xyz(0.75, 0.0, 1.5)
    time.sleep(1.0) 
    bot.move_to_xyz(0.75, 0.0, 1.2)
    time.sleep(1.0) 
    bot.move_to_xyz(0.75, 0.0, 1.5)
    time.sleep(1.0) 
    bot.move_to_xyz(0.75, 0.1, 1.5)
    time.sleep(1.0) 
    bot.move_to_xyz(0.75, 0.1, 1.2)
    
    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()