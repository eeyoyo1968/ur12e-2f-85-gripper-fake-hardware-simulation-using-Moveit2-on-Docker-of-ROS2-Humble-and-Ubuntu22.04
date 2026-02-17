#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
import math

# Import your existing controller
from test_move_xyz_theta_noflip_gmove import UR12eController

class VisionGuidedGrasp(Node):
    def __init__(self):
        super().__init__('vision_guided_grasp')
        
        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Robot controller
        self.robot = UR12eController()
        
        # Subscribe to detected objects
        self.create_subscription(
            PoseStamped,
            '/detected_object_pose',  # Topic from perception
            self.object_callback,
            10
        )
        
        # Grasp parameters
        self.approach_height = 0.15  # 15cm above object
        self.grasp_offset_z = -0.10  # 10cm below camera detection point
        
        # Bin locations (in base_link frame)
        self.bins = {
            'bottle': (0.5, 0.3, 0.4),
            'box': (0.5, -0.3, 0.4),
            'default': (0.6, 0.0, 0.4)
        }
        
        self.get_logger().info("Vision-guided grasp node ready!")
    
    def object_callback(self, msg):
        """Called when perception detects an object."""
        self.get_logger().info(f"Detected object in frame: {msg.header.frame_id}")
        
        # Transform object pose to base_link
        try:
            # Wait for transform (timeout 2 seconds)
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            # Transform the pose
            object_in_base = do_transform_pose(msg.pose, transform)
            
            self.get_logger().info(
                f"Object position in base_link: "
                f"x={object_in_base.position.x:.3f}, "
                f"y={object_in_base.position.y:.3f}, "
                f"z={object_in_base.position.z:.3f}"
            )
            
            # Execute pick and place
            self.pick_and_place(object_in_base, object_class='bottle')
            
        except Exception as e:
            self.get_logger().error(f"Transform failed: {e}")
    
    def pick_and_place(self, object_pose, object_class='default'):
        """Execute pick and place sequence."""
        
        # Extract coordinates
        x = object_pose.position.x
        y = object_pose.position.y
        z = object_pose.position.z + self.grasp_offset_z  # Adjust for gripper
        
        # 1. Move to approach position (above object)
        self.get_logger().info("Moving to approach position...")
        approach_z = z + self.approach_height
        success = self.robot.move_xyz_no_flip(x, y, approach_z)
        if not success:
            self.get_logger().error("Failed to reach approach position")
            return False
        
        # 2. Open gripper
        self.get_logger().info("Opening gripper...")
        self.robot.gripper_move(0.0)
        
        # 3. Lower to grasp position
        self.get_logger().info("Lowering to object...")
        success = self.robot.move_xyz_no_flip(x, y, z)
        if not success:
            self.get_logger().error("Failed to reach object")
            return False
        
        # 4. Close gripper
        self.get_logger().info("Closing gripper...")
        self.robot.gripper_move(0.8)
        
        # 5. Check grasp
        if not self.robot.check_grasp_success():
            self.get_logger().warn("Grasp failed, aborting")
            self.robot.gripper_move(0.0)
            return False
        
        # 6. Lift object
        self.get_logger().info("Lifting object...")
        self.robot.move_xyz_no_flip(x, y, z + self.approach_height)
        
        # 7. Get bin location
        bin_x, bin_y, bin_z = self.bins.get(object_class, self.bins['default'])
        
        # 8. Move to bin approach
        self.get_logger().info(f"Moving to {object_class} bin...")
        self.robot.move_xyz_no_flip(bin_x, bin_y, bin_z + self.approach_height)
        
        # 9. Lower into bin
        self.robot.move_xyz_no_flip(bin_x, bin_y, bin_z)
        
        # 10. Release object
        self.get_logger().info("Releasing object...")
        self.robot.gripper_move(0.0)
        
        # 11. Retreat
        self.robot.move_xyz_no_flip(bin_x, bin_y, bin_z + self.approach_height)
        
        # 12. Return home
        home = [-1.5707, -2.3562, 2.3562, -1.5707, -1.5707, 0.0]
        self.robot.jmove(home)
        
        self.get_logger().info("Pick and place complete!")
        return True

def main():
    rclpy.init()
    node = VisionGuidedGrasp()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.robot.destroy_node()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()