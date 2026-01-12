import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import Constraints, JointConstraint, AttachedCollisionObject, CollisionObject, PositionConstraint, OrientationConstraint
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Pose
import time
import random

class UR12eMasterController(Node):
    def __init__(self):
        super().__init__('ur12e_master_controller')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._cartesian_srv = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        
        self.attach_pub = self.create_publisher(AttachedCollisionObject, '/attached_collision_object', 10)
        self.scene_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.current_joints = {}

        self.get_logger().info('Master Controller Initialized. Waiting for Services...')
        self._action_client.wait_for_server()
        self._cartesian_srv.wait_for_service()

    def joint_cb(self, msg):
        self.current_joints = {n: p for n, p in zip(msg.name, msg.position)}

    def spawn_or_move_box(self, x, y):
        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.id = "target_box"
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.05, 0.05, 0.05]
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = 0.925
        pose.orientation.w = 1.0
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD
        self.scene_pub.publish(obj)

    def move_to_xyz_linear(self, x, y, z):
        """Pure vertical/linear move. High success rate for Z-only changes."""
        target_pose = Pose()
        target_pose.position.x = float(x)
        target_pose.position.y = float(y)
        target_pose.position.z = float(z)
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 1.0
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 0.0

        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        req.group_name = "ur_manipulator"
        req.link_name = "wrist_3_link"
        req.waypoints = [target_pose]
        req.max_step = 0.01 
        req.jump_threshold = 5.0 # Relaxed to prevent math-related failures
        req.avoid_collisions = True

        future = self._cartesian_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        if res and res.fraction > 0.95:
            goal_msg = ExecuteTrajectory.Goal()
            goal_msg.trajectory = res.solution
            send_goal_future = self._execute_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future)
            return True
        else:
            self.get_logger().warn(f"Linear failed ({res.fraction:.2f}). Fallback to BiTRRT.")
            return self.move_to_xyz(x, y, z)

    def move_to_xyz(self, x, y, z, speed=0.2):
        """Standard travel move using BiTRRT (RViz-like)"""
        while not self.current_joints:
            rclpy.spin_once(self)

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.planner_id = "BiTRRT"
        goal_msg.request.max_velocity_scaling_factor = speed
        goal_msg.request.start_state.is_diff = True
        goal_msg.request.num_planning_attempts = 10

        # --- FIX: Path Constraints separated from Goal Constraints ---
        path_constraints = Constraints()
        for j_name in ['wrist_1_joint', 'wrist_3_joint']:
            if j_name in self.current_joints:
                jc = JointConstraint()
                jc.joint_name = j_name
                jc.position = self.current_joints[j_name]
                jc.tolerance_above = 3.15 # Allow half-rotation freedom
                jc.tolerance_below = 3.15
                jc.weight = 1.0
                path_constraints.joint_constraints.append(jc)
        goal_msg.request.path_constraints = path_constraints

        # --- Goal Constraints ---
        goal_c = Constraints()
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.pose.position.x = float(x)
        target_pose.pose.position.y = float(y)
        target_pose.pose.position.z = float(z)
        target_pose.pose.orientation.y = 1.0 # Downward

        pos = PositionConstraint()
        pos.header.frame_id = "world"
        pos.link_name = "wrist_3_link"
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.01] # 1cm tolerance
        pos.constraint_region.primitives.append(s)
        pos.constraint_region.primitive_poses.append(target_pose.pose)
        goal_c.position_constraints.append(pos)

        ori = OrientationConstraint()
        ori.header.frame_id = "world"
        ori.link_name = "wrist_3_link"
        ori.orientation = target_pose.pose.orientation
        ori.absolute_x_axis_tolerance = 0.1
        ori.absolute_y_axis_tolerance = 0.1
        ori.absolute_z_axis_tolerance = 0.1
        goal_c.orientation_constraints.append(ori)

        goal_msg.request.goal_constraints.append(goal_c)
        
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle:
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
        return True

    def send_joint_goal(self, group_name, joints, values, speed=0.1):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = group_name
        goal_msg.request.max_velocity_scaling_factor = speed
        c = Constraints()
        for name, val in zip(joints, values):
            c.joint_constraints.append(JointConstraint(joint_name=name, position=float(val), weight=1.0))
        goal_msg.request.goal_constraints.append(c)
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle:
            rclpy.spin_until_future_complete(self, handle.get_result_async())
        return True

    def manage_attachment(self, object_id, action="attach"):
        att = AttachedCollisionObject()
        att.link_name = "robotiq_85_left_finger_tip_link"
        att.object.id = object_id
        att.object.header.frame_id = att.link_name
        att.touch_links = ["robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link", "target_box"]
        att.object.operation = b'\x00' if action == "attach" else b'\x01'
        self.attach_pub.publish(att)

def main():
    rclpy.init()
    bot = UR12eMasterController()
    grip_j = ['robotiq_85_left_knuckle_joint']

    try:
        while rclpy.ok():
            tx = random.uniform(0.6, 0.8)
            ty = random.uniform(-0.15, 0.15)
            bot.spawn_or_move_box(tx, ty)

            # 1. Approach height (XYZ standard move)
            bot.send_joint_goal('gripper', grip_j, [0.0], speed=0.5)
            bot.move_to_xyz(tx, ty, 1.2) 

            # 2. Linear Descent (Straight down)
            bot.move_to_xyz_linear(tx, ty, 0.98) 

            # 3. Attach
            bot.send_joint_goal('gripper', grip_j, [0.4], speed=0.2)
            bot.manage_attachment("target_box", "attach")
            time.sleep(0.5)

            # 4. Linear Lift (Straight up)
            bot.move_to_xyz_linear(tx, ty, 1.2)

            # 5. Travel to Bin
            bot.move_to_xyz(0.4, 0.4, 1.2) 
            bot.move_to_xyz_linear(0.4, 0.4, 1.0) 

            # 6. Release
            bot.manage_attachment("target_box", "detach")
            bot.send_joint_goal('gripper', grip_j, [0.0], speed=0.5)
            time.sleep(1.0)

    except KeyboardInterrupt:
        pass
    finally:
        # Prevent "shutdown already called" error
        if rclpy.ok():
            bot.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()