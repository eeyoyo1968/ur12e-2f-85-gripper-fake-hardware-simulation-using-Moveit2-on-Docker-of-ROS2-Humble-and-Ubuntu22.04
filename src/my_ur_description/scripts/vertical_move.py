import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, RobotState
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

class UR12eCartesianMove(Node):
    def __init__(self):
        super().__init__('ur12e_cartesian_move')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('Waiting for MoveGroup...')
        self._action_client.wait_for_server()

    def move_vertically(self, x, y, start_z, end_z):
        """Moves the robot straight down while preventing joint wrapping"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.max_velocity_scaling_factor = 0.1 
        
        # --- FIX 1: USE CURRENT STATE AS SEED ---
        # This prevents the 'long way around' rotation
        goal_msg.request.start_state.is_diff = True
        
        # --- FIX 2: OPTIMIZE PLANNING ---
        goal_msg.request.num_planning_attempts = 20
        goal_msg.request.allowed_planning_time = 5.0

        c = Constraints()

        # 1. POSITION: The Target
        pos = PositionConstraint()
        pos.header.frame_id = "world"
        pos.link_name = "wrist_3_link" 
        
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.005] 
        
        target_pose = PoseStamped()
        target_pose.pose.position.x = float(x)
        target_pose.pose.position.y = float(y)
        target_pose.pose.position.z = float(end_z)
        
        pos.constraint_region.primitives.append(s)
        pos.constraint_region.primitive_poses.append(target_pose.pose)
        c.position_constraints.append(pos)

        # 2. ORIENTATION: Strict Downward
        ori = OrientationConstraint()
        ori.header.frame_id = "world"
        ori.link_name = "wrist_3_link"
        ori.orientation.x = 0.0
        ori.orientation.y = 1.0  
        ori.orientation.z = 0.0
        ori.orientation.w = 0.0
        # Tolerances
        ori.absolute_x_axis_tolerance = 0.01
        ori.absolute_y_axis_tolerance = 0.01
        ori.absolute_z_axis_tolerance = 0.01
        ori.weight = 1.0
        c.orientation_constraints.append(ori)

        goal_msg.request.goal_constraints.append(c)

        self.get_logger().info(f"Moving Vertically to X:{x} Y:{y} Z:{end_z}...")
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        handle = send_goal_future.result()
        if handle and handle.accepted:
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            self.get_logger().info("Move Complete!")
        else:
            self.get_logger().error("Goal Rejected. Check if target is out of reach or in collision.")

def main():
    rclpy.init()
    node = UR12eCartesianMove()
    
    # Using your updated test coordinates
    # Ensure (0.7, 0.0) is actually reachable for your UR12e setup
    node.move_vertically(0.7, 0.0, 1.6, 1.1)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()