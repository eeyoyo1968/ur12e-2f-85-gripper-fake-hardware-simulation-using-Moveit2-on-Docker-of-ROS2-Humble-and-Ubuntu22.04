from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import rclpy

class PlanningSceneManager(Node):
    def __init__(self):
        super().__init__('scene_manager')
        # This is the vital connection to MoveIt
        self.publisher = self.create_publisher(CollisionObject, '/collision_object', 10)

    def spawn_box(self):
        obj = CollisionObject()
        obj.header.frame_id = "easydesk"
        obj.id = "workpiece"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.05, 0.05, 0.05]

        pose = Pose()
        pose.position.x = 0.4
        pose.position.y = 0.1
        pose.position.z = 0.05 # 5cm tall box on surface
        pose.orientation.w = 1.0

        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        # Publish the object to the scene
        self.get_logger().info("Publishing box to MoveIt...")
        self.publisher.publish(obj)