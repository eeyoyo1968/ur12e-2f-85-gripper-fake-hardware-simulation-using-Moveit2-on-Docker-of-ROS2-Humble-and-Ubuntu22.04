import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
import sys

class UR12eSmartLock(Node):
    def __init__(self):
        super().__init__('ur12e_smart_lock')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Subscribe to joint states to get live positions
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.current_joints = None
        self.joint_names = ['wrist_1_joint', 'wrist_3_joint']

        self.get_logger().info('Waiting for MoveGroup and Joint States...')
        self._action_client.wait_for_server()

    def joint_cb(self, msg):
        # Map joint names to their current positions
        self.current_joints = {n: p for n, p in zip(msg.name, msg.position)}

    def move_to_xyz_safe(self, x, y, z):
        # Wait until we have joint data
        while self.current_joints is None:
            rclpy.spin_once(self)

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.start_state.is_diff = True
        goal_msg.request.num_planning_attempts = 15
        
        c = Constraints()

        # --- DYNAMIC WRIST LOCKING ---
        # We use a wider tolerance (1.5 rad ~ 85 deg) to avoid "Invalid Goal State"
        for j_name in self.joint_names:
            jc = JointConstraint()
            jc.joint_name = j_name
            jc.position = self.current_joints[j_name]
            jc.tolerance_above = 1.5 
            jc.tolerance_below = 1.5
            jc.weight = 1.0
            c.joint_constraints.append(jc)

        # --- POSITION ---
        pos = PositionConstraint()
        pos.header.frame_id = "world"
        pos.link_name = "wrist_3_link"
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.005] # 5mm tolerance is easier for the solver than 2mm
        
        target_pose = PoseStamped()
        target_pose.pose.position.x = float(x)
        target_pose.pose.position.y = float(y)
        target_pose.pose.position.z = float(z)
        pos.constraint_region.primitives.append(s)
        pos.constraint_region.primitive_poses.append(target_pose.pose)
        c.position_constraints.append(pos)

        # --- ORIENTATION (Down) ---
        ori = OrientationConstraint()
        ori.header.frame_id = "world"
        ori.link_name = "wrist_3_link"
        ori.orientation.y = 1.0
        ori.orientation.w = 0.0
        ori.absolute_x_axis_tolerance = 0.1 # Loosened slightly
        ori.absolute_y_axis_tolerance = 0.1
        ori.absolute_z_axis_tolerance = 0.1
        c.orientation_constraints.append(ori)

        goal_msg.request.goal_constraints.append(c)

        self.get_logger().info(f"Planning to {x}, {y}, {z} using current joints as seeds...")
        
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle.accepted:
            res_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_future)
            return True
        return False

def main():
    rclpy.init()
    bot = UR12eSmartLock()
    
    # Try a coordinate that is definitely reachable (not too far, not too close)
    success = bot.move_to_xyz_safe(1.0, 0.0, 1.2)
    
    if success:
        bot.get_logger().info("Success! Closing program.")
    
    bot.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

if __name__ == '__main__':
    main()