#!/usr/bin/env python3
"""
ROS2 Gripper Bridge for Robotiq 2F-85 via PolyScope X URScript.

PolyScope X uses DISCRETE PRESETS instead of continuous position control:
  rq_move_and_wait(slave_id, preset)
  - preset 0 = Open
  - preset 1 = Close
  - preset 2 = Half-open

Compatible with: PolyScope X 10.x with Robotiq URCapX
Gripper ID: 1 (as detected by Robotiq app)
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
        self.slave_id    = self.declare_parameter('slave_id', 1).value  # Default to 1 for PolyScope X

        # ── Gripper state ───────────────────────────────────────────────
        self.current_position = 0.0
        self._lock = threading.Lock()
        self._gripper_activated = False

        # ── Action CLIENT to ros2_control (for RViz sync) ───────────────
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/robotiq_gripper_controller/follow_joint_trajectory'
        )

        # ── Subscribe to joint states ───────────────────────────────────
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10
        )

        # ── Command subscriber (Float32) ────────────────────────────────
        self.create_subscription(
            Float32,
            '/robotiq_bridge/command',
            self._command_cb,
            10
        )

        # ── Status publisher ────────────────────────────────────────────
        self._status_pub = self.create_publisher(Float32, '/robotiq_bridge/position', 10)
        self.create_timer(0.1, self._publish_status)

        # ── Activate gripper on startup ─────────────────────────────────
        threading.Thread(target=self._startup_activate, daemon=True).start()

        self.get_logger().info(
            f"RobotiqPolyscopeXURScriptServer ready | {self.robot_ip}:{self.script_port}"
        )
        self.get_logger().info("Uses PolyScope X discrete preset API (0=open, 1=close, 2=half)")
        self.get_logger().info(f"Gripper Slave ID: {self.slave_id}")
        self.get_logger().info("Send commands to /robotiq_bridge/command (Float32, 0.0=open 0.8=closed)")

    def _startup_activate(self):
        """Activate gripper once on startup using URScript"""
        time.sleep(2.0)
        self.get_logger().info(f"Activating Robotiq gripper (Slave ID: {self.slave_id})...")

        # PolyScope X: No rq_set_gripper_slave_id function
        # Slave ID is configured in Robotiq app, not via URScript
        activate_script = """def rq_startup():
    textmsg("ROS2: Activating gripper...")
    rq_activate_and_wait()
    textmsg("ROS2: Gripper activated")
end
rq_startup()
"""
        ok = self._send_urscript(activate_script)
        if ok:
            self._gripper_activated = True
            self.get_logger().info("✓ Gripper activated successfully")
        else:
            self.get_logger().error("Gripper activation failed - check URScript port 30002")

    def _joint_state_cb(self, msg: JointState):
        """Track gripper position from joint_states"""
        try:
            idx = msg.name.index('robotiq_85_left_knuckle_joint')
            with self._lock:
                self.current_position = msg.position[idx]
        except ValueError:
            pass

    def _command_cb(self, msg: Float32):
        """Receive gripper command"""
        position_rad = max(0.0, min(0.8, float(msg.data)))
        self.get_logger().info(f"Bridge command received: {position_rad:.3f} rad")
        threading.Thread(target=self._execute_move, args=(position_rad,), daemon=True).start()

    def _execute_move(self, position_rad: float):
        """Execute gripper move via URScript + mirror to ros2_control"""
        if not self._gripper_activated:
            self.get_logger().warn("Gripper not yet activated")

        # Part 1: Physical move via URScript
        urscript_ok = self._send_gripper_urscript(position_rad)
        if not urscript_ok:
            self.get_logger().error("URScript failed")

        # Part 2: Mirror to ros2_control for RViz
        self._send_ros2control_goal(position_rad)

    def _send_gripper_urscript(self, position_rad: float) -> bool:
        """
        Send gripper move via PolyScope X discrete presets.
        
        PolyScope X API: rq_move_and_wait(slave_id, preset)
          preset 0 = Open
          preset 1 = Close
          preset 2 = Half-open (middle position)
        
        We map continuous 0.0-0.8 rad to discrete presets:
          0.0 - 0.2 rad  → preset 0 (open)
          0.3 - 0.5 rad  → preset 2 (half)
          0.6 - 0.8 rad  → preset 1 (close)
        """
        
        # Map radians to discrete preset
        if position_rad < 0.25:
            preset = 0  # Open
            preset_name = "OPEN"
        elif position_rad < 0.55:
            preset = 2  # Half
            preset_name = "HALF"
        else:
            preset = 1  # Close
            preset_name = "CLOSE"

        self.get_logger().info(f"URScript: gripper → preset {preset} ({preset_name}) for {position_rad:.3f} rad")

        # PolyScope X: rq_move_and_wait(slave_id, preset)
        urscript = f"""def rq_move_gripper():
    rq_move_and_wait({self.slave_id}, {preset})
end
rq_move_gripper()
"""
        return self._send_urscript(urscript)

    def _send_urscript(self, script: str) -> bool:
        """Send URScript to port 30002"""
    
        # DEBUG: Print what we're sending
        print("="*60)
        print("SENDING URSCRIPT:")
        print(script)
        print("="*60)
    
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3.0)
        
            print(f"Connecting to {self.robot_ip}:{self.script_port}...")
            sock.connect((self.robot_ip, self.script_port))
            print("Connected!")
        
            sock.sendall(script.encode('utf-8'))
            print("Script sent!")
        
            sock.close()
            print("Socket closed.")
            return True
        except Exception as e:
            self.get_logger().error(f"URScript socket error: {e}")
            print(f"ERROR: {e}")
            return False



    def _send_ros2control_goal(self, position_rad: float):
        """Mirror goal to ros2_control for RViz sync"""
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            return

        point = JointTrajectoryPoint()
        point.positions = [position_rad]
        point.time_from_start = Duration(sec=2, nanosec=0)

        traj = JointTrajectory()
        traj.joint_names = ['robotiq_85_left_knuckle_joint']
        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self._action_client.send_goal_async(goal)

    def _publish_status(self):
        """Publish current gripper position"""
        msg = Float32()
        with self._lock:
            msg.data = float(self.current_position)
        self._status_pub.publish(msg)


def main():
    rclpy.init()
    try:
        node = RobotiqPolyscopeXURScriptServer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
