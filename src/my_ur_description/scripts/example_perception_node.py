import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.tf_broadcaster = TransformBroadcaster(self)
        
    def publish_detected_object(self, obj_class, x_cam, y_cam, z_cam):
        """
        Publish detected object transform.
        x_cam, y_cam, z_cam are in camera optical frame (meters)
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'cam_left_camera_optical_frame'  # Or whichever camera
        t.child_frame_id = f'object_{obj_class}_{int(time.time())}'
        
        # Position from camera
        t.transform.translation.x = float(x_cam)
        t.transform.translation.y = float(y_cam)
        t.transform.translation.z = float(z_cam)
        
        # Identity rotation (or estimated orientation)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)