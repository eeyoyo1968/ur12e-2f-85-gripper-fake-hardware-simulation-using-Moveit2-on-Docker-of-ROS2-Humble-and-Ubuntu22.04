import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import math
import numpy as np

# Messages
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState 

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped # <--- Corrected Import
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest

class UR12eController(Node):
    def __init__(self):
        super().__init__('ur12e_controller')
        
        # Action Clients
        self._arm_client = ActionClient(self, MoveGroup, 'move_action')
        self._gripper_client = ActionClient(self, FollowJointTrajectory, '/robotiq_gripper_controller/follow_joint_trajectory')

        # Feedback variables
        self.current_gripper_pos = 0.0
        
        # Subscriber to Joint States
        self._joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        self.get_logger().info("Connecting to controllers...")
        self._arm_client.wait_for_server()
        self._gripper_client.wait_for_server()
        self.get_logger().info("System Online.")

    def joint_state_callback(self, msg):
        """Updates the gripper knuckle joint position in real-time."""
        joint_name = 'robotiq_85_left_knuckle_joint'
        if joint_name in msg.name:
            idx = msg.name.index(joint_name)
            self.current_gripper_pos = msg.position[idx]

    def gripper_move(self, pos):
        """Moves the gripper and waits for the action to finish."""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['robotiq_85_left_knuckle_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [float(pos)]
        point.time_from_start.sec = 1
        goal_msg.trajectory.points.append(point)
        
        future = self._gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        handle = future.result()
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        
        time.sleep(0.8) # Allow simulation physics to stabilize
        return True

    def check_grasp_success(self):
        """
        Logic: If position > 0.75, the gripper is empty.
        In simulation, this is highly accurate.
        """
        self.get_logger().info(f"Grasp Check - Position: {self.current_gripper_pos:.4f}")
        return self.current_gripper_pos < 0.75

    def jmove(self, jointvector):
        # ... [Your existing jmove code here] ...
        pass


    # *** move to a pose ***
    def move_pose(self, x, y, z, ox, oy, oz, ow, frame_id="base_link"):
        """
        Moves the end-effector relative to frame_id (default 'base_link').
        """
        self.get_logger().info(f"Moving to XYZ: [{x}, {y}, {z}] relative to {frame_id}")
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        
        # 1. Setup Target Pose
        target_pose = Pose()
        target_pose.position = Point(x=float(x), y=float(y), z=float(z))
        target_pose.orientation = Quaternion(x=float(ox), y=float(oy), z=float(oz), w=float(ow))
        
        # 2. Build Constraints
        con = Constraints()
        
        pc = PositionConstraint()
        pc.header.frame_id = frame_id # This determines what (0,0,0) means!
        pc.link_name = "tool0"
        
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.005] # 5mm tolerance for easier planning
        
        pc.constraint_region.primitives.append(sp)
        pc.constraint_region.primitive_poses.append(target_pose)
        pc.weight = 1.0
        con.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = frame_id
        oc.link_name = "tool0"
        oc.orientation = target_pose.orientation
        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01
        oc.weight = 1.0
        con.orientation_constraints.append(oc)

        goal_msg.request.goal_constraints.append(con)
        
        # Set planning parameters
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        
        future = self._arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            return False
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return True

    def move_xyz(self, x, y, z):
        """
        Points vertical downward and moves to XYZ.
        Standard UR 'down' orientation is often [1, 0, 0, 0] 
        meaning 180 degree rotation around X.
        """
        # Adjusted Quaternion for 'Pointing Down'
        # If the robot moves weirdly, we can tune these.
        return self.move_pose(x, y, z, 1.0, 0.0, 0.0, 0.0)

    def move_xyz_base(self, x, y, z):
        """Use this if you measured from the robot's base."""
        return self.move_pose(x, y, z, 1.0, 0.0, 0.0, 0.0, frame_id="base_link")

    def move_xyz_world(self, x, y, z):
        """Use this if you measured from the floor (origin of simulation)."""
        return self.move_pose(x, y, z, 1.0, 0.0, 0.0, 0.0, frame_id="world")

    # *** move to a pose with IK and without flippings ***

    def get_ik(self, x, y, z, frame_id="base_link"):
        """
        Uses MoveIt IK service to find joint angles for an XYZ point
        pointing downward, using the current robot state as a 'seed'.
        """
        # Create a service client for IK
        ik_client = self.create_client(GetPositionIK, 'compute_ik')
        while not ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('IK service not available, waiting...')

        request = GetPositionIK.Request()
        ik_request = PositionIKRequest()
        ik_request.group_name = "ur_manipulator"
        
        # 1. Define the target Pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = frame_id
        target_pose.pose.position = Point(x=float(x), y=float(y), z=float(z))
        # Downward orientation
        target_pose.pose.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        
        ik_request.pose_stamped = target_pose
        
        # 2. SEED STATE: This is the secret to avoiding flips.
        # We tell the solver to find a solution close to our current real position.
        ik_request.robot_state.joint_state.name = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        # You can also pass 'home' vector here to always prefer home-like poses
        ik_request.avoid_collisions = True
        
        request.ik_request = ik_request

        # Call service
        future = ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.error_code.val == 1: # SUCCESS
            return response.solution.joint_state.position
        else:
            self.get_logger().error(f"IK Failed with code: {response.error_code.val}")
            return None


    def move_xyz_no_flip(self, x, y, z, frame_id="base_link"):
        """
        Calculates IK first to ensure a smooth joint-space transition
        without configuration flips.
        """
        self.get_logger().info(f"Calculating smooth IK for XYZ: {x, y, z}")
        
        joint_solution = self.get_ik(x, y, z, frame_id)
        
        if joint_solution is not None:
            # We found a joint vector! Now use jmove to execute it.
            self.get_logger().info("IK Solution found. Executing jmove...")
            return self.jmove(joint_solution)
        else:
            self.get_logger().error("Could not find a valid IK solution for this point.")
            return False






def main():
    # ... init bot ...
    rclpy.init()
    bot = UR12eController()

    # --- Math Constants ---
    PI = math.pi
    D2R = PI / 180.0

    # --- Pose Definitions ---
    # Home (Shoulder at -90 degrees)
    home = [-1.5707, -2.3562, 2.3562, -1.5707, -1.5707, 0.0]


    # 1. Move to a safe "Home" using joints
    bot.jmove(home)

    # 2. Move to a coordinate above the table
    # x=0.5m, y=0.0m, z=0.3m
    bot.get_logger().info("Moving to Cartesian Pick Coordinate")
    bot.move_xyz_base(0.5, 0.0, 0.3) 

    # 3. Lower to pick the object
    bot.move_xyz(0.5, 0.0, 0.15)   # default baselink
    
    # 4. Grasp logic
    bot.gripper_move(0.8)
    if bot.check_grasp_success():
        # Lift up
        bot.move_xyz_base(0.5, 0.0, 0.4)


    # Instead of move_xyz, use the IK-guarded version
    bot.jmove(home) 

    # This will find the joints closest to 'home' that reach this XYZ
    bot.move_xyz_no_flip(0.5, 0.2, 0.3)

    bot.get_logger().info("Shutting down controller.")
    bot.destroy_node()
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()