import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='127.0.0.1',
        description='IP address of URSim'
    )

    robot_ip = LaunchConfiguration('robot_ip')

    # Include main robot launch (use_fake_hardware=false).
    # This spawns robotiq_gripper_controller unconditionally, which is what we
    # want — it publishes gripper joint states to joint_state_broadcaster so
    # RViz always shows the gripper moving.
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('my_ur_description'),
                'launch',
                'my_robot.launch.py'
            )
        ]),
        launch_arguments={
            'use_fake_hardware': 'false',
            'robot_ip': robot_ip,
        }.items()
    )

    # Gripper bridge node.
    # IMPORTANT: The bridge must NOT act as a competing action server for
    # /robotiq_gripper_controller/follow_joint_trajectory. Instead it should
    # forward commands to the UR controller via URScript, while the ros2_control
    # robotiq_gripper_controller handles joint state publishing for RViz.
    # Delayed 8s so robotiq_gripper_controller is fully active before bridge starts.
    gripper_bridge = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='my_ur_description',
                executable='robotiq_ursim_bridge.py',
                name='robotiq_ursim_bridge',
                parameters=[{
                    'robot_ip': robot_ip,
                    # Tells the bridge to forward URScript commands to the UR
                    # controller rather than registering its own action server,
                    # avoiding the double-registration clash.
                    'mode': 'urscript_forward',
                }],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        robot_ip_arg,
        robot_launch,
        gripper_bridge,
    ])