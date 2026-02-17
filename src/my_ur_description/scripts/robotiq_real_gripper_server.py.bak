#!/usr/bin/env python3
"""
ROS2 Action Server for Real Robotiq 2F-85 via UR Controller
Works with both real robot and URSim (with URCap)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
import socket
import time

class RobotiqRealGripperServer(Node):
    def __init__(self):
        super().__init__('robotiq_real_gripper_server')
        
        # Parameters
        self.robot_ip = self.declare_parameter('robot_ip', '192.168.1.100').value
        self.script_port = 30002  # URScript interface
        
        # Gripper state
        self.current_position = 0.0  # 0.0 = open, 0.8 = closed (radians)
        
        # Publisher for gripper state (optional, for monitoring)
        self.state_pub = self.create_publisher(
            JointState,
            '/robotiq_gripper_state',
            10
        )
        
        # Action server - matches your existing controller interface
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/robotiq_gripper_controller/follow_joint_trajectory',
            self.execute_callback
        )
        
        # State publishing timer
        self.timer = self.create_timer(0.1, self.publish_state)
        
        self.get_logger().info(f"Robotiq Real Gripper Server ready (Robot IP: {self.robot_ip})")
        self.get_logger().info("Ensure Robotiq URCap is installed and gripper is activated!")
    
    def send_urscript(self, script):
        """Send URScript commands to UR controller."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2.0)
            s.connect((self.robot_ip, self.script_port))
            
            # Send script
            s.send(script.encode())
            s.close()
            
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to send URScript: {e}")
            return False
    
    def gripper_command(self, position_rad):
        """
        Send gripper command to UR via Robotiq URCap functions.
        position_rad: 0.0 (open) to 0.8 (closed) in radians
        """
        # Convert radians to gripper position (0-255)
        # 0.0 rad = fully open = 0
        # 0.8 rad = fully closed = 255
        gripper_pos = int((position_rad / 0.8) * 255)
        gripper_pos = max(0, min(255, gripper_pos))
        
        self.get_logger().info(f"Commanding gripper to position: {gripper_pos}/255 ({position_rad:.3f} rad)")
        
        # URScript using Robotiq URCap functions
        # These functions are provided by the installed URCap
        ur_script = f"""def gripper_move():
    # Ensure gripper is activated
    rq_activate_and_wait()
    
    # Move to position
    # rq_move_and_wait(position, speed, force)
    # position: 0-255 (0=open, 255=closed)
    # speed: 0-255 (255=max speed)
    # force: 0-255 (255=max force)
    rq_move_and_wait({gripper_pos}, 255, 150)
end

gripper_move()
"""
        
        success = self.send_urscript(ur_script)
        
        if success:
            # Update internal state
            self.current_position = position_rad
        
        return success
    
    def execute_callback(self, goal_handle):
        """Execute gripper trajectory action."""
        request = goal_handle.request
        
        # Extract target position from trajectory
        if len(request.trajectory.points) == 0:
            self.get_logger().error("Empty trajectory received")
            goal_handle.abort()
            return FollowJointTrajectory.Result()
        
        # Get final point
        target_point = request.trajectory.points[-1]
        target_position = target_point.positions[0]  # First joint
        
        self.get_logger().info(f"Executing gripper move to {target_position:.3f} rad")
        
        # Send command
        success = self.gripper_command(target_position)
        
        if not success:
            self.get_logger().error("Failed to command gripper")
            goal_handle.abort()
            return FollowJointTrajectory.Result()
        
        # Wait for movement to complete
        duration = target_point.time_from_start.sec + target_point.time_from_start.nanosec / 1e9
        time.sleep(max(1.0, duration))
        
        # Mark as complete
        goal_handle.succeed()
        
        result = FollowJointTrajectory.Result()
        return result
    
    def publish_state(self):
        """Publish current gripper state."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['robotiq_85_left_knuckle_joint']
        msg.position = [self.current_position]
        msg.velocity = [0.0]
        msg.effort = [0.0]
        
        self.state_pub.publish(msg)

def main():
    rclpy.init()
    server = RobotiqRealGripperServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()