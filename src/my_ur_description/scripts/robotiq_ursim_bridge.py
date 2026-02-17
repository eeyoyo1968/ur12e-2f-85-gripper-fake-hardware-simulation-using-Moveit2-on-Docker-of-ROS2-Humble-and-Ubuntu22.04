#!/usr/bin/env python3
"""
ROS2 Gripper Bridge for URSim / Real UR Controller
Bridges ROS2 gripper commands to UR controller via URScript.

Design: Does NOT register its own action server for
/robotiq_gripper_controller/follow_joint_trajectory.
That action server is owned by the ros2_control spawner in
my_robot.launch.py (always active, keeps RViz joint states alive).

This node subscribes to the gripper controller's joint state output
and forwards URScript commands to the UR controller when a gripper
goal is received — acting as a command forwarder, not a competing server.

Mode:
  'urscript_forward' (default) — forward URScript to UR controller only.
  The ros2_control controller handles the ROS2 action interface and
  joint state publishing.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import socket
import time
import threading


class RobotiqURSimBridge(Node):
    def __init__(self):
        super().__init__('robotiq_ursim_bridge')

        # ── Parameters ──────────────────────────────────────────
        self.robot_ip   = self.declare_parameter('robot_ip',   '127.0.0.1').value
        self.script_port = self.declare_parameter('script_port', 30002).value
        # 'urscript_forward': send URScript AND mirror cmd to ros2_control
        # controller so joint_state_broadcaster sees the position for RViz.
        self.mode = self.declare_parameter('mode', 'urscript_forward').value

        # ── Gripper state ────────────────────────────────────────
        self.current_position = 0.0   # radians: 0.0 open, 0.8 closed
        self._lock = threading.Lock()

        # ── Action CLIENT (not server) ───────────────────────────
        # We send goals TO the ros2_control robotiq_gripper_controller
        # so it updates joint states for RViz. We are NOT a competing server.
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/robotiq_gripper_controller/follow_joint_trajectory'
        )

        # ── Subscribe to gripper joint states for state tracking ─
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10
        )

        # ── Gripper command subscription ─────────────────────────
        # Motion scripts publish here to trigger a gripper move.
        # Topic: /robotiq_bridge/command  (Float32: 0.0–0.8 rad)
        from std_msgs.msg import Float32
        self.create_subscription(
            Float32,
            '/robotiq_bridge/command',
            self._command_cb,
            10
        )

        # ── Status publisher ─────────────────────────────────────
        from std_msgs.msg import Float32 as F32
        self._status_pub = self.create_publisher(F32, '/robotiq_bridge/position', 10)
        self.create_timer(0.1, self._publish_status)

        self.get_logger().info(
            f"RobotiqURSimBridge ready | IP: {self.robot_ip}:{self.script_port} | mode: {self.mode}"
        )
        self.get_logger().info(
            "Send gripper commands to /robotiq_bridge/command (Float32, 0.0=open 0.8=closed)"
        )

    # ── Joint state tracker ──────────────────────────────────────
    def _joint_state_cb(self, msg: JointState):
        try:
            idx = msg.name.index('robotiq_85_left_knuckle_joint')
            with self._lock:
                self.current_position = msg.position[idx]
        except ValueError:
            pass   # joint not in this message

    # ── Command subscriber callback ──────────────────────────────
    def _command_cb(self, msg):
        position_rad = float(msg.data)
        position_rad = max(0.0, min(0.8, position_rad))
        self.get_logger().info(f"Bridge command received: {position_rad:.3f} rad")
        # Run in thread so we don't block the subscriber callback
        t = threading.Thread(target=self._execute_gripper_move, args=(position_rad,), daemon=True)
        t.start()

    # ── Core move logic ──────────────────────────────────────────
    def _execute_gripper_move(self, position_rad: float):
        """
        Two-part move:
          1. Send URScript to UR controller → moves the physical gripper.
          2. Send FollowJointTrajectory goal to ros2_control controller
             → updates joint states so RViz visualises the move.
        """
        # Part 1: URScript to physical gripper
        urscript_ok = self._send_urscript(position_rad)
        if not urscript_ok:
            self.get_logger().error("URScript send failed — physical gripper may not have moved")

        # Part 2: Mirror command to ros2_control controller for RViz
        self._send_ros2control_goal(position_rad)

    def _send_urscript(self, position_rad: float) -> bool:
        """Send URScript gripper command to UR controller port 30002."""
        # Convert radians → Robotiq position integer (0=open, 255=closed)
        gripper_pos = int((position_rad / 0.8) * 255)
        gripper_pos = max(0, min(255, gripper_pos))

        self.get_logger().info(f"URScript: gripper → {gripper_pos}/255")

        # FIX vs original: use rq_activate_and_wait (blocking) not rq_activate
        # to ensure the gripper is ready before commanding movement.
        # Also removed the bare sleep(0.5) which was unreliable.
        ur_script = f"""def rq_bridge_move():
    rq_activate_and_wait()
    rq_move_and_wait({gripper_pos}, 255, 150)
end
rq_bridge_move()
"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3.0)
            s.connect((self.robot_ip, self.script_port))
            s.sendall(ur_script.encode('utf-8'))
            s.close()
            return True
        except Exception as e:
            self.get_logger().error(f"URScript socket error: {e}")
            return False

    def _send_ros2control_goal(self, position_rad: float):
        """
        Mirror the gripper position to the ros2_control controller.
        This keeps joint_state_broadcaster publishing the new gripper
        position so RViz stays in sync with the physical gripper.
        """
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("robotiq_gripper_controller action server not available — RViz may lag")
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
        # Fire-and-forget: we don't block on the result here
        future.add_done_callback(self._ros2control_goal_done)

    def _ros2control_goal_done(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("ros2_control gripper goal rejected")

    # ── Status publisher ─────────────────────────────────────────
    def _publish_status(self):
        from std_msgs.msg import Float32
        msg = Float32()
        with self._lock:
            msg.data = float(self.current_position)
        self._status_pub.publish(msg)


def main():
    rclpy.init()
    node = RobotiqURSimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()