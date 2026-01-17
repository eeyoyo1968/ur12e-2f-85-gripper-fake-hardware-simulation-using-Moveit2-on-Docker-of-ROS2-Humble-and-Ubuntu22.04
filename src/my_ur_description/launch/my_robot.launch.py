import os
import yaml
import math
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, file_path)
    
    def degrees_constructor(loader, node):
        value = loader.construct_scalar(node)
        return math.radians(float(value))

    # Add the constructor to both SafeLoader and Loader to be safe
    yaml.SafeLoader.add_constructor('!degrees', degrees_constructor)

    try:
        with open(absolute_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception: 
        return None

def generate_launch_description():
    description_pkg = "my_ur_description"

    # 1. Declare Launch Arguments
    # We use these variables later in the Condition objects
    use_fake_hardware_config = LaunchConfiguration("use_fake_hardware")
    robot_ip_config = LaunchConfiguration("robot_ip")

    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware", default_value="true"
    )
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip", default_value="192.168.1.100"
    )

    # 2. Robot Description (XACRO)
    xacro_file = PathJoinSubstitution([get_package_share_directory(description_pkg), 'urdf', 'ur_system.xacro'])
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ", xacro_file,
            " ", "use_fake_hardware:=", use_fake_hardware_config,
            " ", "robot_ip:=", robot_ip_config,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # 3. Load Configs
    srdf_file = os.path.join(get_package_share_directory(description_pkg), 'srdf', 'ur_system.srdf')
    with open(srdf_file, 'r') as f:
        semantic_config = f.read()
    robot_description_semantic = {'robot_description_semantic': semantic_config}

    kinematics_yaml = load_yaml(description_pkg, 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml(description_pkg, "config/joint_limits.yaml")
    controllers_yaml = os.path.join(get_package_share_directory(description_pkg), 'config', 'ur_controllers.yaml')
    rviz_config_file = os.path.join(get_package_share_directory(description_pkg), 'rviz', 'my_robot_view.rviz')

    # 4. MoveIt Controller Config (Dynamic based on hardware)
    # Define which controller MoveIt should use
    trajectory_controller_name = "scaled_joint_trajectory_controller"
    if use_fake_hardware_config == "true":
        trajectory_controller_name = "joint_trajectory_controller"

    moveit_controllers = {
        'moveit_simple_controller_manager': {
            # ONLY include the controller that is actually spawned
            'controller_names': [trajectory_controller_name, 'robotiq_gripper_controller'],
            
            trajectory_controller_name: {
                'type': 'FollowJointTrajectory',
                'action_ns': 'follow_joint_trajectory',
                'default': True,
                'joints': ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
            },
            'robotiq_gripper_controller': {
                'type': 'FollowJointTrajectory',
                'action_ns': 'follow_joint_trajectory',
                'default': True,
                'joints': ['robotiq_85_left_knuckle_joint'],
            },
        }
    }
    planning_pipeline_config = {
        'default_planning_pipeline': 'ompl',
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization ' \
                                'default_planner_request_adapters/FixWorkspaceBounds ' \
                                'default_planner_request_adapters/FixStartStateBounds ' \
                                'default_planner_request_adapters/FixStartStateCollision ' \
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
            'longest_valid_segment_fraction': 0.005,
        }
    }

    return LaunchDescription([
        use_fake_hardware_arg,
        robot_ip_arg,

        Node(
            package='robot_state_publisher', 
            executable='robot_state_publisher', 
            output='both', 
            parameters=[robot_description, {'use_sim_time': False}]
        ),
        
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, {'use_sim_time': False, 'update_rate': 100}, controllers_yaml],
            output='both',
        ),

        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                {'robot_description_kinematics': kinematics_yaml},
                planning_pipeline_config,
                moveit_controllers,
                {'use_sim_time': False, 'publish_planning_scene': True, 'robot_description_planning': joint_limits_yaml}
            ],
        ),

        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster']),

        # FIX: Using the variable 'use_fake_hardware_config' which is defined above
        Node(
            package='controller_manager', 
            executable='spawner', 
            arguments=['joint_trajectory_controller'],
            condition=IfCondition(use_fake_hardware_config)
        ),

        Node(
            package='controller_manager', 
            executable='spawner', 
            arguments=['scaled_joint_trajectory_controller'],
            condition=UnlessCondition(use_fake_hardware_config)
        ),

        Node(package='controller_manager', executable='spawner', arguments=['robotiq_gripper_controller']),

        TimerAction(period=5.0, actions=[
            Node(
                package='rviz2', executable='rviz2', name='rviz2', output='screen',
                arguments=['-d', rviz_config_file],
                parameters=[
                    robot_description, 
                    robot_description_semantic, 
                    {'robot_description_kinematics': kinematics_yaml}, 
                    planning_pipeline_config
                ],
            )
        ])
    ])