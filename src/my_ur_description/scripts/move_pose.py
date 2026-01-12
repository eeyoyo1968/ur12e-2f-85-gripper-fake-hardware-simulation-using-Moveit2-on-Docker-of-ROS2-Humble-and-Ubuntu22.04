import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import PoseStamped
import sys

class UR12eDirectMove(Node):
    def __init__(self):
        super().__init__('ur12e_direct_move')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('Connecting to MoveGroup...')
        self._action_client.wait_for_server()

    def move_to_pose(self, x, y, z, ox, oy, oz, ow):
        """Moves to a target pose using the default MoveIt planning pipeline"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        
        # 1. Planning Settings (Optimized for 'Reasonableness')
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        
        # 2. Seed from current state (The 'RViz' Secret)
        goal_msg.request.start_state.is_diff = True

        # 3. Define the Target Pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"
        target_pose.pose.position.x = float(x)
        target_pose.pose.position.y = float(y)
        target_pose.pose.position.z = float(z)
        target_pose.pose.orientation.x = float(ox)
        target_pose.pose.orientation.y = float(oy)
        target_pose.pose.orientation.z = float(oz)
        target_pose.pose.orientation.w = float(ow)

        # 4. Create a Simple Pose Constraint
        # We wrap the pose into a constraint because that's what MoveGroup expects
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        from shape_msgs.msg import SolidPrimitive

        c = Constraints()
        
        # Position part
        pos = PositionConstraint()
        pos.header.frame_id = "world"
        pos.link_name = "wrist_3_link"
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.001] # 1mm tolerance
        pos.constraint_region.primitives.append(s)
        pos.constraint_region.primitive_poses.append(target_pose.pose)
        c.position_constraints.append(pos)
        
        # Orientation part
        ori = OrientationConstraint()
        ori.header.frame_id = "world"
        ori.link_name = "wrist_3_link"
        ori.orientation = target_pose.pose.orientation
        ori.absolute_x_axis_tolerance = 0.001
        ori.absolute_y_axis_tolerance = 0.001
        ori.absolute_z_axis_tolerance = 0.001
        c.orientation_constraints.append(ori)

        goal_msg.request.goal_constraints.append(c)

        self.get_logger().info(f"Direct Move to Pose: {x, y, z}")
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        handle = send_goal_future.result()
        if handle.accepted:
            res_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_future)
            self.get_logger().info("Target reached successfully.")
        return True

def main():
    rclpy.init()
    bot = UR12eDirectMove()
    
    # Example: Move to a specific point with "Gripper Down" orientation
    # X=0.5, Y=0.0, Z=1.2
    # Orientation (Quaternion for downward): x=0, y=1, z=0, w=0
    bot.move_to_pose(0.9, 0.0, 1.4, -0.7071, 0.0, 0.0, 0.7071)
    
    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()