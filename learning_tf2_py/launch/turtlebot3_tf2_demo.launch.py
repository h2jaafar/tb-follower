from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    return LaunchDescription([
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_target_broadcaster',
            name='broadcaster',
        ),
    ])