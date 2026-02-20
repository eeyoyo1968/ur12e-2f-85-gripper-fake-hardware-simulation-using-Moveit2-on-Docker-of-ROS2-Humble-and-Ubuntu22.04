#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import socket
import threading
import time

class RobotiqPolyscopeXURScriptServer(Node):
    def __init__(self):
        super().__init__('robotiq_polyscope_x_urscript_server')

        self.robot_ip    = self.declare_parameter('robot_ip', '192.168.2.2').value
        self.script_port = self.declare_parameter('script_port', 30002).value
        self.slave_id    = self.declare_parameter('slave_id', 1).value 

        self.current_position = 0.0
        self.last_logic_state = None 
        self._lock = threading.Lock()
        self._gripper_activated = False

        self.create_subscription(JointState, '/joint_states', self._sync_with_robot_moves, 10)
        
        threading.Thread(target=self._startup_activate, daemon=True).start()
        self.get_logger().info(f"Bridge Active. IP: {self.robot_ip}. PLEASE SWITCH TO REMOTE MODE.")

    def _sync_with_robot_moves(self, msg: JointState):
        try:
            idx = msg.name.index('robotiq_85_left_knuckle_joint')
            target_pos = msg.position[idx]
            
            # 0 = Open, 1 = Close
            new_logic_state = 1 if target_pos >= 0.4 else 0
            
            if new_logic_state != self.last_logic_state:
                self.last_logic_state = new_logic_state
                state_name = "CLOSE" if new_logic_state == 1 else "OPEN"
                self.get_logger().info(f"Targeting State: {state_name}")
                
                # We attempt to move. If not activated yet, it will just log a warning.
                threading.Thread(target=self._execute_move, args=(target_pos,), daemon=True).start()
        except ValueError:
            pass

    def _startup_activate(self):
        """Standard Robotiq Activation Sequence"""
        time.sleep(1.0) # Wait for node to settle
        script = f"def rq_activate():\n  rq_set_gripper_slave_id({self.slave_id})\n  rq_activate_and_wait()\nend\nrq_activate()\n"
        
        if self._send_urscript(script):
            self._gripper_activated = True
            self.get_logger().info("✓ Gripper Activated via URScript.")
            # Trigger an immediate move to whatever the current ROS2 state is
            self._execute_move(self.current_position) 
        else:
            self.get_logger().error("Failed to connect to robot for activation.")

    def _execute_move(self, target_rad: float):
        if not self._gripper_activated:
            self.get_logger().warn("Move skipped: Gripper not yet activated.")
            return

        # Map to your working commands
        if target_rad < 0.4:
            cmd, label = "rq_open_and_wait()", "OPENING"
        else:
            cmd, label = "rq_close_and_wait()", "CLOSING"

        urscript = f"""def rq_move():
    rq_set_gripper_slave_id({self.slave_id})
    textmsg("ROS2: {label}")
    {cmd}
end
rq_move()
"""
        self._send_urscript(urscript)

    def _send_urscript(self, script: str) -> bool:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(2.0)
                sock.connect((self.robot_ip, self.script_port))
                sock.sendall(script.encode('utf-8'))
            return True
        except Exception:
            return False

def main():
    rclpy.init()
    node = RobotiqPolyscopeXURScriptServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Prevent the 'RCLError: failed to shutdown' message
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()