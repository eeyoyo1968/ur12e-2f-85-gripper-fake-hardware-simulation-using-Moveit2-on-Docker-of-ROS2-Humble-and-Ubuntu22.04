import rclpy
from rclpy.node import Node
# This is the most likely package for the robot_mode topic
try:
    from ur_dashboard_msgs.msg import RobotMode
except ImportError:
    # Fallback for some driver versions
    from ur_msgs.msg import RobotMode 

class TableToucher(Node):
    def __init__(self):
        super().__init__('table_toucher')
        
        self.subscription = self.create_subscription(
            RobotMode,
            '/io_and_status_controller/robot_mode',
            self.mode_callback,
            10)
        
        self.last_mode = None
        self.get_logger().info("--- TABLE TOUCH MONITOR ACTIVE ---")
        self.get_logger().info("Watching for transition from 7 (Running) to 3 (Stop/Contact)")

    def mode_callback(self, msg):
        current_mode = msg.mode
        
        # Only log when the mode actually changes to avoid spam
        if current_mode != self.last_mode:
            if current_mode == 3 and self.last_mode == 7:
                self.get_logger().error(">>>> CONTACT DETECTED: PROTECTIVE STOP TRIGGERED <<<<")
            else:
                self.get_logger().info(f"Robot Mode: {current_mode}")
            
            self.last_mode = current_mode

def main(args=None):
    rclpy.init(args=args)
    node = TableToucher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()