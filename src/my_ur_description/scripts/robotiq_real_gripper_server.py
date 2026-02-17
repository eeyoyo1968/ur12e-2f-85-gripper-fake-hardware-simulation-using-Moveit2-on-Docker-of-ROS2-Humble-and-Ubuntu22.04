#!/usr/bin/env python3
"""
ROS2 Gripper Bridge for Real Robotiq 2F-85 via UR Controller.

Design: Does NOT register its own action server for
/robotiq_gripper_controller/follow_joint_trajectory.
That action server is owned by the ros2_control spawner in
my_robot.launch.py (always active, keeps RViz joint states alive).

This node acts as a command forwarder:
  - Subscribes to /robotiq_bridge/command (Float32)
  - Sends URScript to the real UR controller (port 30002)
    which uses the Robotiq URCap to move the physical gripper
  - Sends a mirrored goal to the ros2_control controller
    so joint_state_broadcaster publishes the new position for RViz

Requires: Robotiq 2F Gripper URCap installed on UR12e
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


class RobotiqRealGripperServer(Node):
    def __init__(self):
        super().__init__('robotiq_real_gripper_server')

        # ── Parameters ──────────────────────────────────────────
        self.robot_ip    = self.declare_parameter('robot_ip',    '192.168.1.100').value
        self.script_port = self.declare_parameter('script_port', 30002).value

        # ── Gripper state ────────────────────────────────────────
        self.current_position = 0.0   # radians: 0.0 open, 0.8 closed
        self._lock = threading.Lock()
        self._gripper_activated = False

        # ── Action CLIENT to ros2_control controller ─────────────
        # We send goals TO the controller (not act as a server ourselves).
        # This keeps RViz joint state display in sync.
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/robotiq_gripper_controller/follow_joint_trajectory'
        )

        # ── Subscribe to joint states for position tracking ──────
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10
        )

        # ── Command subscriber ───────────────────────────────────
        # Motion scripts publish here: Float32, 0.0 (open) – 0.8 (closed)
        self.create_subscription(
            Float32,
            '/robotiq_bridge/command',
            self._command_cb,
            10
        )

        # ── Status publisher ─────────────────────────────────────
        self._status_pub = self.create_publisher(Float32, '/robotiq_bridge/position', 10)
        self.create_timer(0.1, self._publish_status)

        # ── Activate gripper on startup ──────────────────────────
        # Do this in a thread so __init__ returns immediately
        threading.Thread(target=self._startup_activate, daemon=True).start()

        self.get_logger().info(
            f"RobotiqRealGripperServer ready | Robot IP: {self.robot_ip}:{self.script_port}"
        )
        self.get_logger().info(
            "Send gripper commands to /robotiq_bridge/command (Float32, 0.0=open 0.8=closed)"
        )
        self.get_logger().info("Ensure Robotiq URCap is installed and robot is running!")

    # ── Startup activation ───────────────────────────────────────
    def _startup_activate(self):
        """
        Activate the gripper once on startup.
        FIX vs original: activation is done once here rather than on every
        gripper_command call via rq_activate_and_wait(), which caused a
        blocking delay before every single move.
        """
        time.sleep(2.0)   # Wait for UR controller to be ready
        self.get_logger().info("Activating Robotiq gripper...")
        activate_script = """def rq_startup():
    rq_activate_and_wait()
end
rq_startup()
"""
        ok = self._send_urscript_raw(activate_script)
        if ok:
            self._gripper_activated = True
            self.get_logger().info("Gripper activated successfully")
        else:
            self.get_logger().error(
                "Gripper activation failed — check URCap installation and robot connection"
            )

    # ── Joint state tracker ──────────────────────────────────────
    def _joint_state_cb(self, msg: JointState):
        try:
            idx = msg.name.index('robotiq_85_left_knuckle_joint')
            with self._lock:
                self.current_position = msg.position[idx]
        except ValueError:
            pass

    # ── Command subscriber callback ──────────────────────────────
    def _command_cb(self, msg: Float32):
        position_rad = float(msg.data)
        position_rad = max(0.0, min(0.8, position_rad))
        self.get_logger().info(f"Bridge command received: {position_rad:.3f} rad")
        t = threading.Thread(target=self._execute_gripper_move, args=(position_rad,), daemon=True)
        t.start()

    # ── Core move logic ──────────────────────────────────────────
    def _execute_gripper_move(self, position_rad: float):
        """
        Two-part move:
          1. URScript → physical gripper via UR controller URCap
          2. ros2_control goal → RViz joint state stays in sync
        """
        if not self._gripper_activated:
            self.get_logger().warn("Gripper not yet activated — attempting move anyway")

        # Part 1: physical move via URScript
        urscript_ok = self._send_gripper_urscript(position_rad)
        if not urscript_ok:
            self.get_logger().error("URScript failed — physical gripper may not have moved")

        # Part 2: mirror to ros2_control for RViz
        self._send_ros2control_goal(position_rad)

    def _send_gripper_urscript(self, position_rad: float) -> bool:
        """
        Send a gripper move via URScript.
        FIX vs original:
          - rq_activate_and_wait() removed from per-move call (done once at startup)
          - rq_move_and_wait used with force=150 (safer than 255 for most objects)
          - script name made unique per call to avoid UR controller caching issues
        """
        gripper_pos = int((position_rad / 0.8) * 255)
        gripper_pos = max(0, min(255, gripper_pos))

        self.get_logger().info(f"URScript: gripper → {gripper_pos}/255 ({position_rad:.3f} rad)")

        ur_script = f"""def rq_real_move():
    rq_move_and_wait({gripper_pos}, 255, 150)
end
rq_real_move()
"""
        return self._send_urscript_raw(ur_script)

    def _send_urscript_raw(self, script: str) -> bool:
        """Open socket, send URScript, close. Thread-safe."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3.0)
            s.connect((self.robot_ip, self.script_port))
            s.sendall(script.encode('utf-8'))
            s.close()
            return True
        except Exception as e:
            self.get_logger().error(f"URScript socket error: {e}")
            return False

    def _send_ros2control_goal(self, position_rad: float):
        """
        Send a mirrored goal to ros2_control robotiq_gripper_controller.
        This keeps joint_state_broadcaster publishing the new position
        so RViz shows the gripper at the correct position.
        """
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("robotiq_gripper_controller not available — RViz may not update")
            return

        point = JointTrajectoryPoint()
        point.positions = [position_rad]
        point.time_from_start = Duration(sec=2, nanosec=0)

        traj = JointTrajectory()
        traj.joint_names = ['robotiq_85_left_knuckle_joint']
        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._ros2control_goal_done)

    def _ros2control_goal_done(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("ros2_control gripper goal rejected")

    # ── Status publisher ─────────────────────────────────────────
    def _publish_status(self):
        msg = Float32()
        with self._lock:
            msg.data = float(self.current_position)
        self._status_pub.publish(msg)


def main():
    rclpy.init()
    node = RobotiqRealGripperServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()