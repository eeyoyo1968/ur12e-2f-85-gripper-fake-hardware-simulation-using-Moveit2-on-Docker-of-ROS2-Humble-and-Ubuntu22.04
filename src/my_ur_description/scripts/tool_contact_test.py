import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ContactMonitor(Node):
    def __init__(self):
        super().__init__('contact_monitor')
        self.sub = self.create_subscription(JointState, '/joint_states', self.cb, 10)
        self.names_printed = False
        self.get_logger().info("Discovery mode active. Monitoring ALL non-joint states...")

    def cb(self, msg):
        # 1. Print all names once so we know what we are looking at
        if not self.names_printed:
            self.get_logger().info(f"Available names in joint_states: {msg.name}")
            self.names_printed = True

        # 2. Check for the tool contact specifically
        target = 'tool_contact/tool_contact_state'
        if target in msg.name:
            idx = msg.name.index(target)
            # Check all arrays (position, velocity, effort)
            for array in [msg.position, msg.velocity, msg.effort]:
                if idx < len(array) and array[idx] > 0.5:
                    self.get_logger().warn("!!! CONTACT DETECTED !!!")

def main():
    rclpy.init()
    node = ContactMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Avoid the double shutdown error
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()