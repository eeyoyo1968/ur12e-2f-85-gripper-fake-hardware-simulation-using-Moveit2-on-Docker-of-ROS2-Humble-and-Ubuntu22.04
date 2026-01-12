import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from geometry_msgs.msg import Pose
import time

class UR12eCartesianController(Node):
    def __init__(self):
        super().__init__('ur12e_cartesian_controller')
        self._cartesian_srv = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self._exec_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        
        self._cartesian_srv.wait_for_service()
        self._exec_client.wait_for_server()

        # Your exact orientation
        self.qx, self.qy, self.qz, self.qw = -0.707, 0.707, 0.001, 0.000

    def move_to_xyz_linear(self, x, y, z):
        """Computes and executes a straight-line Cartesian path"""
        target_pose = Pose()
        target_pose.position.x = float(x)
        target_pose.position.y = float(y)
        target_pose.position.z = float(z)
        target_pose.orientation.x = self.qx
        target_pose.orientation.y = self.qy
        target_pose.orientation.z = self.qz
        target_pose.orientation.w = self.qw

        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        req.group_name = "ur_manipulator"
        req.waypoints = [target_pose]
        req.max_step = 0.01          # 1cm resolution for smoothness
        req.jump_threshold = 0.0      # Set to 0.0 to disable jump checks or ~5.0 for safety
        req.avoid_collisions = True

        self.get_logger().info(f"Computing Cartesian path to: {x}, {y}, {z}")
        future = self._cartesian_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        # Check if the solver successfully planned the path (fraction = 1.0 means 100% success)
        if res and res.fraction > 0.95:
            self.get_logger().info(f"Path success ({res.fraction*100}%). Executing...")
            goal_msg = ExecuteTrajectory.Goal()
            goal_msg.trajectory = res.solution
            
            send_goal_future = self._exec_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future)
            return True
        else:
            self.get_logger().error(f"Cartesian path failed! Only {res.fraction*100}% planned.")
            return False

def main():
    rclpy.init()
    bot = UR12eCartesianController()
    
    # Linear move from current position to a new target

    bot.move_to_xyz_linear(0.75, 0.0, 1.5)
    time.sleep(1.0) 
    bot.move_to_xyz_linear(0.75, 0.0, 1.2)
    time.sleep(1.0) 
    bot.move_to_xyz_linear(0.75, 0.0, 1.5)
    time.sleep(1.0) 
    bot.move_to_xyz_linear(0.75, 0.1, 1.5)
    time.sleep(1.0) 
    bot.move_to_xyz_linear(0.75, 0.1, 1.2)
    
    
    bot.destroy_node()
    rclpy.shutdown()