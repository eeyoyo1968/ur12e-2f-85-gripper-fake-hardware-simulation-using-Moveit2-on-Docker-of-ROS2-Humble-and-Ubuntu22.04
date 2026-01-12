import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
import time

class UR12eFlexibleMove(Node):
    def __init__(self):
        super().__init__('ur12e_flexible_move')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self._action_client.wait_for_server()
        
        # Captured orientation from your TF
        self.qx, self.qy, self.qz, self.qw = -0.707, 0.707, 0.001, 0.000

    def move_to_xyz(self, x, y, z):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.num_planning_attempts = 15
        goal_msg.request.allowed_planning_time = 2.0
        goal_msg.request.max_velocity_scaling_factor = 0.1

        # Relaxed Constraints
        c = Constraints()

        # 1. Position (Target XYZ)
        pos = PositionConstraint()
        pos.header.frame_id = "world"
        pos.link_name = "wrist_3_link"
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.01] # Allow 1cm tolerance
        
        target_pose = PoseStamped()
        target_pose.pose.position.x = float(x)
        target_pose.pose.position.y = float(y)
        target_pose.pose.position.z = float(z)
        
        pos.constraint_region.primitives.append(s)
        pos.constraint_region.primitive_poses.append(target_pose.pose)
        c.position_constraints.append(pos)

        # 2. Orientation (RELAXED)
        # Instead of 0.001, we use 0.1 to give the IK solver 'room' to find a solution
        ori = OrientationConstraint()
        ori.header.frame_id = "world"
        ori.link_name = "wrist_3_link"
        ori.orientation.x = self.qx
        ori.orientation.y = self.qy
        ori.orientation.z = self.qz
        ori.orientation.w = self.qw
        ori.absolute_x_axis_tolerance = 0.1 
        ori.absolute_y_axis_tolerance = 0.1
        ori.absolute_z_axis_tolerance = 0.1
        c.orientation_constraints.append(ori)

        goal_msg.request.goal_constraints.append(c)

        self.get_logger().info(f"Flexible move to: {x}, {y}, {z}")
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle and handle.accepted:
            res_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_future)
            return True
        return False

def main():
    rclpy.init()
    bot = UR12eFlexibleMove()
    
    # Try the point that failed before
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