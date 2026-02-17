#!/usr/bin/env python3
"""
Test camera-robot calibration by pointing at a known location.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class CalibrationTest(Node):
    def __init__(self):
        super().__init__('calibration_test')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
    def test_point(self, x_cam, y_cam, z_cam, camera_frame='cam_left_camera_optical_frame'):
        """Transform a point from camera to base_link."""
        
        point_in_cam = PointStamped()
        point_in_cam.header.frame_id = camera_frame
        point_in_cam.header.stamp = self.get_clock().now().to_msg()
        point_in_cam.point.x = x_cam
        point_in_cam.point.y = y_cam
        point_in_cam.point.z = z_cam
        
        try:
            # Transform to base_link
            point_in_base = self.tf_buffer.transform(
                point_in_cam, 
                'base_link',
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            self.get_logger().info(
                f"Camera: ({x_cam:.3f}, {y_cam:.3f}, {z_cam:.3f}) → "
                f"Base: ({point_in_base.point.x:.3f}, {point_in_base.point.y:.3f}, "
                f"{point_in_base.point.z:.3f})"
            )
            
            return point_in_base
            
        except Exception as e:
            self.get_logger().error(f"Transform failed: {e}")
            return None

def main():
    rclpy.init()
    node = CalibrationTest()
    
    # Test: Object 50cm in front of camera (Z-axis)
    # Camera at desk position (0.2, -0.4, 0.2)
    # Camera looking at (rpy="0 0.5 1.57")
    
    node.test_point(0.0, 0.0, 0.5)  # 50cm forward from camera
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()