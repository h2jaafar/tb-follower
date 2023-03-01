from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '0', '0', 'world', 'odom']
        ),
        Node(
            package='learning_tf2_py',
            executable='turtlebot_tf2_target_broadcaster',
            name='broadcaster',
        ),
        Node(
            package='learning_tf2_py',
            executable='turtlebot_tf2_target_listener',
            name='listener',
        ),
    ])