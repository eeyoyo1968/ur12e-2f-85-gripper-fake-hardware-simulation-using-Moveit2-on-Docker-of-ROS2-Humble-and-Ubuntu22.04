import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
from launch.actions import TimerAction

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: 
        return None

def generate_launch_description():
    description_pkg = "my_ur_description"
    
    # 1. URDF
    xacro_file = os.path.join(get_package_share_directory(description_pkg), 'urdf', 'ur_system.xacro')
    
    # Remove the 'mappings' dictionary entirely
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 2. SRDF
    srdf_file = os.path.join(get_package_share_directory(description_pkg), 'srdf', 'ur_system.srdf')
    with open(srdf_file, 'r') as f:
        semantic_config = f.read()
    robot_description_semantic = {'robot_description_semantic': semantic_config}

    # 3. Kinematics
    kinematics_yaml = load_yaml(description_pkg, 'config/kinematics.yaml')
    kinematics_parameters = {'robot_description_kinematics': kinematics_yaml}
    rviz_kinematics_config = {'robot_description_kinematics': kinematics_yaml}
    
    controllers_yaml = os.path.join(get_package_share_directory(description_pkg), 'config', 'ur_controllers.yaml')

    joint_limits_yaml = load_yaml("my_ur_description", "config/joint_limits.yaml")
    # Then add {'robot_description_planning': joint_limits_yaml} to the parameters list
    
    # 4. RViz Configuration File Path
    rviz_config_file = os.path.join(
        get_package_share_directory(description_pkg), 'rviz', 'my_robot_view.rviz'
    )

    # 5. MoveIt Controller Config
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

    # 6. Planning Pipeline
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

    # Define RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description, 
            robot_description_semantic, 
            kinematics_yaml,
            rviz_kinematics_config,
            planning_pipeline_config
        ],
    )

    return LaunchDescription([
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
                {'robot_description': robot_description_config.toxml(), 
                 'use_sim_time': False,
                 'update_rate': 100},
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
                kinematics_parameters,
                planning_pipeline_config,
                moveit_controllers,
                {
                    'use_sim_time': False,
                    'monitor_dynamics': False,
                    'publish_planning_scene': True,
                    'joint_state_monitor.max_joint_state_age': 0.0, 
                    'wait_for_initial_state_timeout': 20.0,
                    'trajectory_execution.check_starting_state': False,
                    'trajectory_execution.allowed_start_tolerance': 0.0,
                    'trajectory_execution.execution_duration_monitoring': False,
                    'planning_time': 10.0,
                    'planning_attempts': 10,
                    'robot_description_planning': joint_limits_yaml
                }
            ],
        ),

        Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster']),
        Node(package='controller_manager', executable='spawner', arguments=['joint_trajectory_controller']),
        Node(package='controller_manager', executable='spawner', arguments=['robotiq_gripper_controller']),

        TimerAction(period=5.0, actions=[rviz_node])
    ])