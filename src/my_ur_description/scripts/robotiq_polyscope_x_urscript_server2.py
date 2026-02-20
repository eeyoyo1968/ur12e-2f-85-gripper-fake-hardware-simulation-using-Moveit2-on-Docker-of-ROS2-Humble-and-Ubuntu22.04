#!/usr/bin/env python3
"""
ROS2 Gripper Bridge for Robotiq 2F-85 via PolyScope X URScript.
Modified to intercept FollowJointTrajectory goals and sync them to URScript commands.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float32
import socket
import threading
import time

class RobotiqPolyscopeXURScriptServer(Node):
    def __init__(self):
        super().__init__('robotiq_polyscope_x_urscript_server')

        # ── Parameters ──────────────────────────────────────────────────
        self.robot_ip    = self.declare_parameter('robot_ip', '192.168.2.2').value
        self.script_port = self.declare_parameter('script_port', 30002).value
        self.slave_id    = self.declare_parameter('slave_id', 1).value 

        # ── Gripper state ───────────────────────────────────────────────
        self.current_position = 0.0
        self.last_sent_pos = -1.0
        self._lock = threading.Lock()
        self._gripper_activated = False

        # ── Action CLIENT for RViz synchronization ──────────────────────
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/robotiq_gripper_controller/follow_joint_trajectory'
        )

        # ── Subscriptions ───────────────────────────────────────────────
        # This is the critical part: it listens to the joint states produced 
        # by your test_move script to trigger the physical URScript
        self.create_subscription(JointState, '/joint_states', self._sync_with_robot_moves, 10)
        
        # Legacy support for direct Float32 commands
        self.create_subscription(Float32, '/robotiq_bridge/command', self._command_cb, 10)
        
        # ── Status publisher ────────────────────────────────────────────
        self._status_pub = self.create_publisher(Float32, '/robotiq_bridge/position', 10)
        self.create_timer(0.1, self._publish_status)

        # Start activation thread
        threading.Thread(target=self._startup_activate, daemon=True).start()

        self.get_logger().info(f"Node Started. Target Robot IP: {self.robot_ip}")

    def _sync_with_robot_moves(self, msg: JointState):
        """
        Intercepts joint movements from the move script and sends 
        the corresponding URScript to the physical robot.
        """
        try:
            idx = msg.name.index('robotiq_85_left_knuckle_joint')
            target_pos = msg.position[idx]
            
            # Update internal state
            with self._lock:
                self.current_position = target_pos
            
            # Change detection: only send if target shifted significantly (e.g., from 0.0 to 0.8)
            # This prevents flooding the robot with socket requests
            if abs(target_pos - self.last_sent_pos) > 0.1:
                self.get_logger().info(f"Detected Moveit/Action movement to: {target_pos:.2f}")
                self.last_sent_pos = target_pos
                threading.Thread(target=self._execute_move, args=(target_pos,), daemon=True).start()
                
        except ValueError:
            pass

    def _startup_activate(self):
        """Initializes the gripper using the validated rq_activate_and_wait"""
        time.sleep(2.0)
        self.get_logger().info("Activating gripper via URScript...")
        
        activate_script = f"""def rq_startup():
    textmsg("ROS2: Setting Slave ID {self.slave_id}")
    rq_set_gripper_slave_id({self.slave_id})
    textmsg("ROS2: Activating Gripper...")
    rq_activate_and_wait()
    textmsg("ROS2: Activation Complete")
end
rq_startup()
"""
        if self._send_urscript(activate_script):
            self._gripper_activated = True
            self.get_logger().info("✓ Gripper activation script sent")
        else:
            self.get_logger().error("Activation failed - check connection to robot")

    def _command_cb(self, msg: Float32):
        """Standard command callback for Float32 inputs"""
        self._execute_move(msg.data)

    def _execute_move(self, target_rad: float):
        """Sends the physical URScript command to the robot and logs to pendant."""
        if not self._gripper_activated:
            self.get_logger().warn("Gripper not activated; command ignored.")
            return

        # Binary logic for open/close as requested
        if target_rad < 0.4:
            script_cmd = "rq_open_and_wait()"
            log_label = "OPENING"
        else:
            script_cmd = "rq_close_and_wait()"
            log_label = "CLOSING"

        urscript = f"""def rq_move_gripper():
    rq_set_gripper_slave_id({self.slave_id})
    textmsg("ROS2: {log_label}...")
    {script_cmd}
    textmsg("ROS2: Done {log_label}")
end
rq_move_gripper()
"""
        self.get_logger().info(f"Sending URScript to {self.robot_ip}:\n{urscript}")
        self._send_urscript(urscript)

    def _send_urscript(self, script: str) -> bool:
        """Lower level socket communication to port 30002"""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(3.0)
                sock.connect((self.robot_ip, self.script_port))
                sock.sendall(script.encode('utf-8'))
            return True
        except Exception as e:
            self.get_logger().error(f"Socket error on {self.robot_ip}: {e}")
            return False

    def _publish_status(self):
        msg = Float32()
        with self._lock:
            msg.data = float(self.current_position)
        self._status_pub.publish(msg)

def main():
    rclpy.init()
    node = RobotiqPolyscopeXURScriptServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()