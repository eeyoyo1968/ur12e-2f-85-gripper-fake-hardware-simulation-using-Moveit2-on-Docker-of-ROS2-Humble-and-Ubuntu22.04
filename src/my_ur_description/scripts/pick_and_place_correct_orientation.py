import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, AttachedCollisionObject, CollisionObject
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive
import time
import random

class UR12eFinalController(Node):
    def __init__(self):
        super().__init__('ur12e_final_controller')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._cartesian_srv = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        
        self.attach_pub = self.create_publisher(AttachedCollisionObject, '/attached_collision_object', 10)
        self.scene_pub = self.create_publisher(CollisionObject, '/collision_object', 10)

        # YOUR TESTED ORIENTATION (Cleaned up)
        self.target_qx = -0.707
        self.target_qy = 0.707
        self.target_qz = 0.001
        self.target_qw = 0.000

        self.get_logger().info('Connecting to MoveIt...')
        self._action_client.wait_for_server()
        self._cartesian_srv.wait_for_service()

    def move_to_pose(self, x, y, z, speed=0.2):
        """Standard Pose Goal - Matches RViz dragging behavior"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.planner_id = "BiTRRT" # Smoother than RRTConnect
        goal_msg.request.max_velocity_scaling_factor = speed
        goal_msg.request.start_state.is_diff = True

        c = Constraints()
        
        # Target Pose using your captured TF values
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.pose.position.x = float(x)
        target_pose.pose.position.y = float(y)
        target_pose.pose.position.z = float(z)
        target_pose.pose.orientation.x = self.target_qx
        target_pose.pose.orientation.y = self.target_qy
        target_pose.pose.orientation.z = self.target_qz
        target_pose.pose.orientation.w = self.target_qw

        # Position Constraint
        pos = PositionConstraint()
        pos.header.frame_id = "world"
        pos.link_name = "wrist_3_link"
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.001] 
        pos.constraint_region.primitives.append(s)
        pos.constraint_region.primitive_poses.append(target_pose.pose)
        c.position_constraints.append(pos)

        # Orientation Constraint
        ori = OrientationConstraint()
        ori.header.frame_id = "world"
        ori.link_name = "wrist_3_link"
        ori.orientation = target_pose.pose.orientation
        ori.absolute_x_axis_tolerance = 0.01
        ori.absolute_y_axis_tolerance = 0.01
        ori.absolute_z_axis_tolerance = 0.01
        c.orientation_constraints.append(ori)

        goal_msg.request.goal_constraints.append(c)
        
        self.get_logger().info(f"Moving to XYZ: {x:.2f}, {y:.2f}, {z:.2f}")
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle:
            rclpy.spin_until_future_complete(self, handle.get_result_async())
        return True

    def move_linear_z(self, x, y, z_target):
        """Pure vertical motion for picking/placing"""
        target_pose = Pose()
        target_pose.position.x = float(x)
        target_pose.position.y = float(y)
        target_pose.position.z = float(z_target)
        target_pose.orientation.x = self.target_qx
        target_pose.orientation.y = self.target_qy
        target_pose.orientation.z = self.target_qz
        target_pose.orientation.w = self.target_qw

        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        req.group_name = "ur_manipulator"
        req.link_name = "wrist_3_link"
        req.waypoints = [target_pose]
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        future = self._cartesian_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        if res and res.fraction > 0.9:
            goal_msg = ExecuteTrajectory.Goal()
            goal_msg.trajectory = res.solution
            self._execute_client.send_goal_async(goal_msg)
            time.sleep(1.0) # Simple wait for linear execution
            return True
        return self.move_to_pose(x, y, z_target)

def main():
    rclpy.init()
    bot = UR12eFinalController()
    
    try:
        while rclpy.ok():
            # Random target
            tx, ty = random.uniform(0.6, 0.8), random.uniform(-0.15, 0.15)
            
            # 1. Move ABOVE target (using RViz-style Pose Goal)
            bot.move_to_pose(tx, ty, 1.3)
            
            # 2. Linear DROP (Straight line)
            bot.move_linear_z(tx, ty, 1.1)
            
            # 3. Linear LIFT
            bot.move_linear_z(tx, ty, 1.3)
            
            # 4. Move to BIN
            bot.move_to_pose(0.4, 0.4, 1.3)
            
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()