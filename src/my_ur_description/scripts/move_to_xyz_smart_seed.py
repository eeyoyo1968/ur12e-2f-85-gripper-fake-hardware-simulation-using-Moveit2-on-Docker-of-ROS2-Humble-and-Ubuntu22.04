import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.srv import GetPositionIK, GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import time

class UR12eSmartController(Node):
    def __init__(self):
        super().__init__('ur12e_smart_controller')
        self._ik_srv = self.create_client(GetPositionIK, 'compute_ik')
        self._cartesian_srv = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self._exec_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.current_joint_state = None
        
        self._ik_srv.wait_for_service()
        self._cartesian_srv.wait_for_service()
        self._exec_client.wait_for_server()
        self._move_group_client.wait_for_server()

        # Your exact orientation
        self.qx, self.qy, self.qz, self.qw = -0.707, 0.707, 0.001, 0.000

    def joint_cb(self, msg):
        self.current_joint_state = msg

    def move_to_xyz(self, x, y, z):
        # 1. Wait for joints
        while self.current_joint_state is None:
            self.get_logger().info("Waiting for joint states...")
            rclpy.spin_once(self, timeout_sec=0.1)

        target_pose = Pose()
        target_pose.position.x, target_pose.position.y, target_pose.position.z = float(x), float(y), float(z)
        target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w = self.qx, self.qy, self.qz, self.qw

        # 2. Try Cartesian (Straight Line)
        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        req.group_name = "ur_manipulator"
        req.waypoints = [target_pose]
        req.max_step = 0.01          
        req.jump_threshold = 5.0 # Increased for better linear success
        req.avoid_collisions = True

        future = self._cartesian_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        if res and res.fraction > 0.90:
            self.get_logger().info(f"Cartesian path {res.fraction*100:.1f}% found. Executing...")
            return self.execute_traj(res.solution)

        # 3. Fallback 1: Seeded IK (Locked Config)
        self.get_logger().warn("Linear path failed. Trying Locked-Config IK...")
        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = "ur_manipulator"
        ik_req.ik_request.pose_stamped.header.frame_id = "world"
        ik_req.ik_request.pose_stamped.pose = target_pose
        ik_req.ik_request.robot_state.joint_state = self.current_joint_state
        ik_req.ik_request.avoid_collisions = True
        
        ik_future = self._ik_srv.call_async(ik_req)
        rclpy.spin_until_future_complete(self, ik_future)
        ik_res = ik_future.result()

        if ik_res.error_code.val == 1:
            self.get_logger().info("Locked-Config found. Moving...")
            return self.execute_joint_goal(ik_res.solution.joint_state)

        # 4. Fallback 2: Relaxed IK (Global Search - The "RViz" way)
        self.get_logger().error("Locked-Config unreachable. Trying Global IK Search...")
        ik_req.ik_request.robot_state = RobotState() # Clear seed to allow any configuration
        
        ik_future_global = self._ik_srv.call_async(ik_req)
        rclpy.spin_until_future_complete(self, ik_future_global)
        ik_res_global = ik_future_global.result()

        if ik_res_global.error_code.val == 1:
            self.get_logger().info("Global IK found (Configuration may change).")
            return self.execute_joint_goal(ik_res_global.solution.joint_state)
        
        self.get_logger().error("Point completely unreachable!")
        return False

    def execute_joint_goal(self, joint_state):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.max_velocity_scaling_factor = 0.1
        c = Constraints()
        for name, pos in zip(joint_state.name, joint_state.position):
            if "robotiq" not in name:
                jc = JointConstraint(joint_name=name, position=pos, tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
                c.joint_constraints.append(jc)
        goal_msg.request.goal_constraints.append(c)
        future = self._move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle.accepted:
            rclpy.spin_until_future_complete(self, handle.get_result_async())
            return True
        return False

    def execute_traj(self, traj):
        goal_msg = ExecuteTrajectory.Goal(trajectory=traj)
        future = self._exec_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle.accepted:
            rclpy.spin_until_future_complete(self, handle.get_result_async())
            return True
        return False

def main():
    rclpy.init()
    bot = UR12eSmartController()
    
    targets = [(0.75, 0.0, 1.2), (0.75, 0.0, 1.5), (0.75, 0.0, 1.5), (0.75, 0.1, 1.5), (0.75, 0.1, 1.2)]

    for x, y, z in targets:
        bot.move_to_xyz(x, y, z)
        time.sleep(0.5)

    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()