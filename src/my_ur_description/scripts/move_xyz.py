import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

class UR12eSimpleMove(Node):
    def __init__(self):
        super().__init__('ur12e_simple_move')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('Waiting for MoveGroup...')
        self._action_client.wait_for_server()

    def move_to_xyz(self, x, y, z):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.max_velocity_scaling_factor = 0.2
        goal_msg.request.allowed_planning_time = 5.0

        # 1. Setup the Constraints Container
        c = Constraints()
        
        # 2. Position Constraint (Target point)
        pos = PositionConstraint()
        pos.header.frame_id = "world"
        # Using wrist_3_link is usually more stable than the finger tips for initial testing
        pos.link_name = "wrist_3_link" 
        
        # Tolerance sphere of 1cm around the target
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.01] 
        
        target_pose = PoseStamped()
        target_pose.pose.position.x = float(x)
        target_pose.pose.position.y = float(y)
        target_pose.pose.position.z = float(z)
        
        pos.constraint_region.primitives.append(s)
        pos.constraint_region.primitive_poses.append(target_pose.pose)
        c.position_constraints.append(pos)

        # 3. Orientation Constraint (Pointing Vertically Downward)
        # To point down, we rotate 180 degrees (PI) around the Y-axis
        ori = OrientationConstraint()
        ori.header.frame_id = "world"
        ori.link_name = "wrist_3_link"
        ori.orientation.x = 0.0
        ori.orientation.y = 1.0 # Standard "Down" orientation for UR robots
        ori.orientation.z = 0.0
        ori.orientation.w = 0.0
        ori.absolute_x_axis_tolerance = 0.1
        ori.absolute_y_axis_tolerance = 0.1
        ori.absolute_z_axis_tolerance = 0.1
        c.orientation_constraints.append(ori)

        goal_msg.request.goal_constraints.append(c)

        self.get_logger().info(f"Sending Goal: X={x}, Y={y}, Z={z}")
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        handle = send_goal_future.result()
        if handle and handle.accepted:
            self.get_logger().info("Goal Accepted. Moving...")
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            self.get_logger().info("Move Complete!")
        else:
            self.get_logger().error("Goal Rejected by MoveGroup.")

def main():
    rclpy.init()
    node = UR12eSimpleMove()
    
    # TEST COORDINATES: 40cm forward, 0cm side, 1.1m high (safely above the 0.9m desk)
    node.move_to_xyz(0.8, 0.0, 1.1)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()