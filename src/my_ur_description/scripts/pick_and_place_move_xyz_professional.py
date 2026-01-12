import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, 
    JointConstraint, 
    PositionConstraint, 
    OrientationConstraint, 
    BoundingVolume
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class UR12eFinalPicker(Node):
    def __init__(self):
        # FIX: Ensure the node name is passed correctly here
        super().__init__('ur12e_final_picker')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info("Waiting for MoveGroup...")
        self._action_client.wait_for_server()

    def move_to_home(self):
        self.get_logger().info("Moving to HOME to satisfy constraints...")
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        
        js = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        con = Constraints()
        for name, pos in zip(names, js):
            jc = JointConstraint(joint_name=name, position=pos, 
                                 tolerance_above=0.1, tolerance_below=0.1, weight=1.0)
            con.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(con)
        return self._send_goal(goal_msg)

    def move_to_pose(self, x, y, z):
        self.get_logger().info(f"Computing Cartesian path to ({x}, {y}, {z})...")
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        
        # Define the target pose
        target_pose = Pose()
        target_pose.position.x = float(x)
        target_pose.position.y = float(y)
        target_pose.position.z = float(z)
        # Gripper pointing down
        target_pose.orientation.x = -0.707
        target_pose.orientation.y = 0.707
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 0.0

        # We will use the 'compute_cartesian_path' service logic via the MoveGroup action
        # by providing the target as the only waypoint in a constrained request.
        
        goal_constraints = Constraints()
        
        # Position Constraint (Tight)
        pos_con = PositionConstraint()
        pos_con.header.frame_id = "world"
        pos_con.link_name = "tool0"
        s = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01, 0.01, 0.01])
        bv = BoundingVolume()
        bv.primitives.append(s)
        bv.primitive_poses.append(target_pose)
        pos_con.constraint_region = bv
        pos_con.weight = 1.0

        # Orientation Constraint
        ori_con = OrientationConstraint()
        ori_con.header.frame_id = "world"
        ori_con.link_name = "tool0"
        ori_con.orientation = target_pose.orientation
        ori_con.absolute_x_axis_tolerance = 0.1
        ori_con.absolute_y_axis_tolerance = 0.1
        ori_con.absolute_z_axis_tolerance = 0.1
        ori_con.weight = 1.0

        goal_constraints.position_constraints.append(pos_con)
        goal_constraints.orientation_constraints.append(ori_con)
        
        goal_msg.request.goal_constraints.append(goal_constraints)
        
        # Reduce the max velocity/acceleration for safety during Cartesian moves
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1

        return self._send_goal(goal_msg)

    def _send_goal(self, goal_msg):
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted: return False
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return True

def main():
    rclpy.init()
    bot = UR12eFinalPicker()
    
    bot.move_to_home()
    time.sleep(5.0)
    targets = [(0.7, 0.0, 1.2), (0.7, 0.1, 1.2), (0.7, -0.1, 1.2)]

    for x, y, z in targets:
        print("target")
        bot.move_to_pose(x, y, z)
        time.sleep(5.0)

    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()