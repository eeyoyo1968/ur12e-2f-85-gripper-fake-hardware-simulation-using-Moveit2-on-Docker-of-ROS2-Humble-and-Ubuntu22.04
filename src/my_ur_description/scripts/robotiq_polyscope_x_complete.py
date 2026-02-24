#!/usr/bin/env python3
"""
ROS2 Gripper Bridge for Robotiq 2F-85 via PolyScope X.

Sends the complete Robotiq function library with each command.
This is required because PolyScope X doesn't have built-in rq_* functions.

Compatible with: PolyScope X 10.x
Gripper ID: 1
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


# Complete Robotiq function library (from Gripper.script)
ROBOTIQ_LIBRARY = """
# Robotiq Gripper Function Library
ACT = 1
GTO = 2
ATR = 3
ARD = 4
FOR = 5
SPE = 6
OBJ = 7
STA = 8
FLT = 9
POS = 10
PRE = 11

def rq_init_connection(gripper_sid=1, gripper_socket="1"):
    socket_open("127.0.0.1",63352, gripper_socket)
    socket_set_var("SID", gripper_sid,  gripper_socket)
    ack = socket_read_byte_list(3, gripper_socket)
end

def rq_set_var(var_name, var_value, gripper_socket="1"):
    sync()
    if (var_name == ACT):
        socket_set_var("ACT", var_value, gripper_socket)
    elif (var_name == GTO):
        socket_set_var("GTO", var_value, gripper_socket)
    elif (var_name == POS):
        socket_set_var("POS", var_value, gripper_socket)
    elif (var_name == SPE):
        socket_set_var("SPE", var_value, gripper_socket)
    elif (var_name == FOR):
        socket_set_var("FOR", var_value, gripper_socket)
    end
    sync()
    ack = socket_read_byte_list(3, gripper_socket)
end

def rq_get_var(var_name, nbr_bytes, gripper_socket="1"):
    if (var_name == STA):
        socket_send_string("GET STA",gripper_socket)
    elif (var_name == OBJ):
        socket_send_string("GET OBJ",gripper_socket)
    elif (var_name == PRE):
        socket_send_string("GET PRE",gripper_socket)
    end
    sync()
    var_value = socket_read_byte_list(nbr_bytes, gripper_socket)
    sync()
    return var_value
end

def is_STA_gripper_activated(list_of_bytes):
    if (list_of_bytes[0] != 1):
        return False
    end
    if (list_of_bytes[1] == 51):
        return True
    end
    return False
end

def is_OBJ_object_detected(list_of_bytes):
    if (list_of_bytes[0] != 1):
        return False
    end
    if (list_of_bytes[1] == 50 or list_of_bytes[1] == 49):
        return True
    end
    return False
end

def is_OBJ_gripper_at_position(list_of_bytes):
    if (list_of_bytes[0] != 1):
        return False
    end
    if (list_of_bytes[1] == 51):
        return True
    end
    return False
end

def rq_reset(gripper_socket="1"):
    rq_set_var(ACT,0, gripper_socket)
    rq_set_var(ATR,0, gripper_socket)
end

def rq_activate(gripper_socket="1"):
    if (not rq_is_gripper_activated(gripper_socket)):
        rq_reset(gripper_socket)
    end
    rq_set_var(ACT,1, gripper_socket)
end

def rq_is_gripper_activated(gripper_socket="1"):
    gSTA = rq_get_var(STA, 1, gripper_socket)
    if(is_STA_gripper_activated(gSTA)):
        return True
    else:
        return False
    end
end

def rq_activate_and_wait(gripper_socket="1"):
    rq_activate(gripper_socket)
    while(not rq_is_gripper_activated(gripper_socket)):
        sleep(0.1)
    end
end

def rq_set_pos(pos, gripper_socket="1"):
    rq_set_var(GTO,0, gripper_socket)
    rq_set_var(POS, pos, gripper_socket)
    gPRE = rq_get_var(PRE, 3, gripper_socket)
    pre = (gPRE[1] - 48)*100 + (gPRE[2] -48)*10 + gPRE[3] - 48
    sync()
    while (pre != pos):
        rq_set_var(POS, pos, gripper_socket)
        gPRE = rq_get_var(PRE, 3, gripper_socket)
        pre = (gPRE[1] - 48)*100 + (gPRE[2] -48)*10 + gPRE[3] - 48
        sync()
    end
end

def rq_go_to(gripper_socket="1"):
    rq_set_var(GTO,1, gripper_socket)
end

def rq_is_motion_complete(gripper_socket="1"):
    gOBJ = rq_get_var(OBJ, 1, gripper_socket)
    sleep(0.01)
    if (is_OBJ_gripper_at_position(gOBJ)):
        return True
    end
    if (is_OBJ_object_detected(gOBJ)):
        return True
    end
    return False
end

def rq_move(pos, gripper_socket="1"):
    rq_set_pos(pos, gripper_socket)
    rq_go_to(gripper_socket)
end

def rq_move_and_wait(pos, gripper_socket="1"):
    rq_move(pos, gripper_socket)
    while (not rq_is_motion_complete(gripper_socket)):
        sleep(0.01)
        sync()
    end
end

def rq_open_and_wait(gripper_socket="1"):
    rq_move_and_wait(0, gripper_socket)
end

def rq_close_and_wait(gripper_socket="1"):
    rq_move_and_wait(255, gripper_socket)
end
"""


class RobotiqPolyscopeXComplete(Node):
    def __init__(self):
        super().__init__('robotiq_polyscope_x_complete')

        self.robot_ip    = self.declare_parameter('robot_ip', '192.168.2.2').value
        self.script_port = self.declare_parameter('script_port', 30002).value
        self.slave_id    = self.declare_parameter('slave_id', 1).value

        self.current_position = 0.0
        self._lock = threading.Lock()
        self._gripper_activated = False

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/robotiq_gripper_controller/follow_joint_trajectory'
        )

        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)
        self.create_subscription(Float32, '/robotiq_bridge/command', self._command_cb, 10)

        self._status_pub = self.create_publisher(Float32, '/robotiq_bridge/position', 10)
        self.create_timer(0.1, self._publish_status)

        threading.Thread(target=self._startup_activate, daemon=True).start()

        self.get_logger().info(f"RobotiqPolyscopeXComplete ready | {self.robot_ip}:{self.script_port}")
        self.get_logger().info(f"Gripper Slave ID: {self.slave_id}")
        self.get_logger().info("Uses complete Robotiq function library (0-255 position)")

    def _startup_activate(self):
        time.sleep(2.0)
        self.get_logger().info("Activating Robotiq gripper with function library...")

        script = f"""def gripper_activate():
{ROBOTIQ_LIBRARY}
    rq_init_connection({self.slave_id}, "1")
    rq_activate_and_wait("1")
end
gripper_activate()
"""
        ok = self._send_urscript(script)
        if ok:
            self._gripper_activated = True
            self.get_logger().info("✓ Gripper activated successfully")
        else:
            self.get_logger().error("Gripper activation failed")

    def _joint_state_cb(self, msg: JointState):
        try:
            idx = msg.name.index('robotiq_85_left_knuckle_joint')
            with self._lock:
                self.current_position = msg.position[idx]
        except ValueError:
            pass

    def _command_cb(self, msg: Float32):
        position_rad = max(0.0, min(0.8, float(msg.data)))
        self.get_logger().info(f"Bridge command received: {position_rad:.3f} rad")
        threading.Thread(target=self._execute_move, args=(position_rad,), daemon=True).start()

    def _execute_move(self, position_rad: float):
        if not self._gripper_activated:
            self.get_logger().warn("Gripper not yet activated")

        urscript_ok = self._send_gripper_urscript(position_rad)
        if not urscript_ok:
            self.get_logger().error("URScript failed")

        self._send_ros2control_goal(position_rad)

    def _send_gripper_urscript(self, position_rad: float) -> bool:
        # Convert radians to Robotiq position (0-255)
        gripper_pos = int((position_rad / 0.8) * 255)
        gripper_pos = max(0, min(255, gripper_pos))

        self.get_logger().info(f"URScript: gripper → {gripper_pos}/255 ({position_rad:.3f} rad)")

        # Send complete library + move command
        script = f"""def gripper_move():
{ROBOTIQ_LIBRARY}
    rq_init_connection({self.slave_id}, "1")
    rq_move_and_wait({gripper_pos}, "1")
end
gripper_move()
"""
        return self._send_urscript(script)

    def _send_urscript(self, script: str) -> bool:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3.0)
            sock.connect((self.robot_ip, self.script_port))
            sock.sendall(script.encode('utf-8'))
            sock.close()
            return True
        except Exception as e:
            self.get_logger().error(f"URScript socket error: {e}")
            return False

    def _send_ros2control_goal(self, position_rad: float):
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
        msg = Float32()
        with self._lock:
            msg.data = float(self.current_position)
        self._status_pub.publish(msg)


def main():
    rclpy.init()
    try:
        node = RobotiqPolyscopeXComplete()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
