import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
import sys

class UR12eRVizMove(Node):
    def __init__(self):
        super().__init__('ur12e_rviz_move')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('Connecting to MoveGroup...')
        self._action_client.wait_for_server()

    def move_xyz(self, x, y, z):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        
        # RViz-like planning settings
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.2
        
        # Seed with current state to avoid ridiculous jumps
        goal_msg.request.start_state.is_diff = True

        # --- GOAL CONSTRAINTS (Where we want to go) ---
        c = Constraints()
        
        # 1. Position
        pos = PositionConstraint()
        pos.header.frame_id = "world"
        pos.link_name = "wrist_3_link"
        
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.01] # 1cm tolerance (RViz style)
        
        target_pose = PoseStamped()
        target_pose.pose.position.x = float(x)
        target_pose.pose.position.y = float(y)
        target_pose.pose.position.z = float(z)
        
        pos.constraint_region.primitives.append(s)
        pos.constraint_region.primitive_poses.append(target_pose.pose)
        c.position_constraints.append(pos)

        # 2. Orientation (Pointing Down)
        ori = OrientationConstraint()
        ori.header.frame_id = "world"
        ori.link_name = "wrist_3_link"
        ori.orientation.x = 0.0
        ori.orientation.y = 1.0
        ori.orientation.z = 0.0
        ori.orientation.w = 0.0
        ori.absolute_x_axis_tolerance = 0.1
        ori.absolute_y_axis_tolerance = 0.1
        ori.absolute_z_axis_tolerance = 0.1
        c.orientation_constraints.append(ori)

        goal_msg.request.goal_constraints.append(c)

        self.get_logger().info(f"Executing RViz-style move to: {x}, {y}, {z}")
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        handle = send_goal_future.result()
        if handle.accepted:
            res_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_future)
            self.get_logger().info("Target Reached.")
            return True
        else:
            self.get_logger().error("Planning Failed.")
            return False

def main():
    rclpy.init()
    bot = UR12eRVizMove()
    
    # Coordinates to test
    success = bot.move_xyz(0.6, 0.0, 1.2)
    
    bot.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

if __name__ == '__main__':
    main()