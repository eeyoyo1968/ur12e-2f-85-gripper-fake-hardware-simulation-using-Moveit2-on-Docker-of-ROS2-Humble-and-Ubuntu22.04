import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution

import math # Add this import at the top

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, file_path)
    
    # Define a constructor for the custom '!degrees' tag
    def degrees_constructor(loader, node):
        value = loader.construct_scalar(node)
        return math.radians(float(value))

    # Register the constructor
    yaml.SafeLoader.add_constructor('!degrees', degrees_constructor)

    try:
        with open(absolute_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: 
        return None

        
def generate_launch_description():
    description_pkg = "my_ur_description"

    # 1. Declare Launch Arguments
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware", default_value="true"
    )
    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip", default_value="192.168.1.100"
    )

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")

    # 2. Process XACRO using Command Substitution (Fixes the TypeError)
    xacro_file = PathJoinSubstitution([get_package_share_directory(description_pkg), 'urdf', 'ur_system.xacro'])
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ", xacro_file,
            " ", "use_fake_hardware:=", use_fake_hardware,
            " ", "robot_ip:=", robot_ip,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # 3. Load Other Configs
    srdf_file = os.path.join(get_package_share_directory(description_pkg), 'srdf', 'ur_system.srdf')
    with open(srdf_file, 'r') as f:
        semantic_config = f.read()
    robot_description_semantic = {'robot_description_semantic': semantic_config}

    kinematics_yaml = load_yaml(description_pkg, 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml(description_pkg, "config/joint_limits.yaml")
    controllers_yaml = os.path.join(get_package_share_directory(description_pkg), 'config', 'ur_controllers.yaml')
    rviz_config_file = os.path.join(get_package_share_directory(description_pkg), 'rviz', 'my_robot_view.rviz')

    # 4. MoveIt Configs
    moveit_controllers = {
        'moveit_simple_controller_manager': {
            'controller_names': ['joint_trajectory_controller', 'robotiq_gripper_controller'],
            'joint_trajectory_controller': {
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

    # 5. Nodes
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
            parameters=[
                robot_description, 
                {'use_sim_time': False, 'update_rate': 100},
                controllers_yaml
            ],
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
                {
                    'use_sim_time': False,
                    'publish_planning_scene': True,
                    'robot_description_planning': joint_limits_yaml
                }
            ],
        ),

        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster']),
        Node(package='controller_manager', executable='spawner', arguments=['joint_trajectory_controller']),
        Node(package='controller_manager', executable='spawner', arguments=['robotiq_gripper_controller']),

        TimerAction(period=5.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
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