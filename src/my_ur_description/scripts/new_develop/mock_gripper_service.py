import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool # Simple placeholder for Open/Close

class MockGripper(Node):
    def __init__(self):
        super().__init__('mock_gripper_service')
        self.srv = self.create_service(SetBool, 'control_gripper', self.handle_gripper)
        self.get_logger().info('Mock Gripper Service Ready (Waiting for Commands)')

    def handle_gripper(self, request, response):
        action = "Closing" if request.data else "Opening"
        self.get_logger().info(f'Gripper: {action}...')
        
        # Simulate real mechanical delay
        import time
        time.sleep(1.2) 
        
        response.success = True
        response.message = f"Gripper successfully {'closed' if request.data else 'opened'}"
        return response

def main():
    rclpy.init()
    node = MockGripper()
    rclpy.spin(node)
    rclpy.shutdown()