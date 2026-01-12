import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.srv import GetPositionIK, GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
import time

class UR12eSteadyController(Node):
    def __init__(self):
        super().__init__('ur12e_steady_controller')
        self._ik_srv = self.create_client(GetPositionIK, 'compute_ik')
        self._cartesian_srv = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self._exec_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.current_joint_state = None
        
        self._ik_srv.wait_for_service()
        self._cartesian_srv.wait_for_service()
        self._exec_client.wait_for_server()

        # Your exact orientation
        self.qx, self.qy, self.qz, self.qw = -0.707, 0.707, 0.001, 0.000

    def joint_cb(self, msg):
        self.current_joint_state = msg

    def move_to_xyz_steady(self, x, y, z):
        """Attempts Cartesian first, then Seeded Joint move to prevent flipping"""
        if self.current_joint_state is None:
            self.get_logger().warn("No joint states yet...")
            return False

        target_pose = Pose()
        target_pose.position.x, target_pose.position.y, target_pose.position.z = float(x), float(y), float(z)
        target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w = self.qx, self.qy, self.qz, self.qw

        # 1. ATTEMPT CARTESIAN (The smoothest move)
        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        req.group_name = "ur_manipulator"
        req.waypoints = [target_pose]
        req.max_step = 0.01          
        req.jump_threshold = 5.0 # Allow small jumps to avoid spinning
        req.avoid_collisions = True

        future = self._cartesian_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        if res and res.fraction > 0.95:
            self.get_logger().info("Executing Cartesian Path...")
            return self.execute_trajectory(res.solution)

        # 2. FALLBACK: SEEDED IK (Prevents shoulder-left/right flips)
        self.get_logger().warn(f"Cartesian failed ({res.fraction*100:.1f}%). Solving Seeded IK...")
        
        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = "ur_manipulator"
        ik_req.ik_request.pose_stamped.header.frame_id = "world"
        ik_req.ik_request.pose_stamped.pose = target_pose
        ik_req.ik_request.avoid_collisions = True
        
        # This SEED is what stops the shoulder flip
        ik_req.ik_request.robot_state.joint_state = self.current_joint_state
        
        ik_future = self._ik_srv.call_async(ik_req)
        rclpy.spin_until_future_complete(self, ik_future)
        ik_res = ik_future.result()

        if ik_res.error_code.val == 1:
            self.get_logger().info("Seeded IK found. Moving to joints...")
            return self.execute_joint_goal(ik_res.solution.joint_state)
        else:
            self.get_logger().error("Even Seeded IK failed. Target might be out of reach.")
            return False

    def execute_joint_goal(self, joint_state):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.max_velocity_scaling_factor = 0.1
        
        # Enforce the specific joint solution found by IK
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
        
        future = self._move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle.accepted:
            rclpy.spin_until_future_complete(self, handle.get_result_async())
            return True
        return False

    def execute_trajectory(self, traj):
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = traj
        future = self._exec_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle.accepted:
            rclpy.spin_until_future_complete(self, handle.get_result_async())
            return True
        return False

def main():
    rclpy.init()
    bot = UR12eSteadyController()
    
    # Run a few spins to get joint state
    for _ in range(5): rclpy.spin_once(bot, timeout_sec=0.1)

    targets = [(0.75, 0.0, 1.5), (0.75, 0.0, 1.2), (0.75, 0.0, 1.5), (0.75, 0.1, 1.5), (0.75, 0.1, 1.2)]

    for x, y, z in targets:
        bot.move_to_xyz_steady(x, y, z)
        time.sleep(0.5) 
    
    bot.destroy_node()
    rclpy.shutdown()