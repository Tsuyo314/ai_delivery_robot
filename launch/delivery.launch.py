import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory('ai_delivery_robot')
    config = os.path.join(package_dir, 'config', 'delivery.yaml')
    world = os.path.join(package_dir, 'worlds', 'delivery_world.sdf')
    physical_world = os.path.join(
        package_dir, 'worlds', 'delivery_world_physical.sdf'
    )
    physical_delivery = LaunchConfiguration('physical_delivery')
    gz_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
    )
    bridge_args = [
        '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
        '/bumper/base@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        '/bumper/left_wheel@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        '/bumper/right_wheel@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        '/world/delivery_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
    ]
    return LaunchDescription([
        DeclareLaunchArgument(
            'physical_delivery', default_value='false',
            description='Push dynamic packages instead of using virtual pickup.',
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(package_dir, 'models'),
        ),
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=os.path.join(package_dir, 'models'),
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                 launch_arguments={'gz_args': f'-r {world}'}.items(),
                                 condition=UnlessCondition(physical_delivery)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                 launch_arguments={'gz_args': f'-r {physical_world}'}.items(),
                                 condition=IfCondition(physical_delivery)),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=bridge_args,
             parameters=[{
                 'qos_overrides./cmd_vel.subscriber.reliability': 'reliable',
             }],
             output='screen'),
        Node(package='ai_delivery_robot', executable='perception_node', output='screen'),
        Node(package='ai_delivery_robot', executable='planner_node', output='screen'),
        Node(package='ai_delivery_robot', executable='navigation_node', output='screen',
             parameters=[config]),
        Node(package='ai_delivery_robot', executable='delivery_manager_node', output='screen',
             parameters=[
                 config,
                 {'physical_delivery': ParameterValue(
                     physical_delivery, value_type=bool)},
             ]),
    ])
