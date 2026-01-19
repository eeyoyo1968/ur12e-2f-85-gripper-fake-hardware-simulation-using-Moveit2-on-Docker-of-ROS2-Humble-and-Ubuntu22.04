import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

class RobotReset(Node):
    def __init__(self):
        super().__init__('robot_reset_node')
        
        # Action Clients
        self._gripper_client = ActionClient(self, FollowJointTrajectory, '/robotiq_gripper_controller/follow_joint_trajectory')
        self._arm_client = ActionClient(self, MoveGroup, 'move_action')

        self.get_logger().info("Resetting Robot to Safe State...")
        self._gripper_client.wait_for_server()
        self._arm_client.wait_for_server()

    def open_gripper(self):
        self.get_logger().info("1. Opening Gripper...")
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['robotiq_85_left_knuckle_joint']
        point = JointTrajectoryPoint()
        point.positions = [0.0]  # Fully Open
        point.time_from_start.sec = 1
        goal_msg.trajectory.points.append(point)
        
        future = self._gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        return True

    def move_home(self):
        self.get_logger().info("2. Moving Arm to Safe Home...")
        home_joints = [-1.5707, -2.3562, 2.3562, -1.5707, -1.5707, 0.0]
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        
        joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        con = Constraints()
        for name, pos in zip(joint_names, home_joints):
            jc = JointConstraint(joint_name=name, position=float(pos), 
                                 tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            con.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(con)
        future = self._arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Reset Complete.")
        return True

def main():
    rclpy.init()
    reset_node = RobotReset()
    reset_node.open_gripper()
    reset_node.move_home()
    rclpy.shutdown()

if __name__ == '__main__':
    main()