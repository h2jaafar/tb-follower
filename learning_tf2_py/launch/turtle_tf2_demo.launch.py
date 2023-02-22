from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    return LaunchDescription([
        # Node(
        #     package='turtlesim',
        #     executable='turtlesim_node',
        #     name='sim',
        #     parameters=[
        #         {'use_sim_time': 'False'}]
        # ),
        DeclareLaunchArgument(
            'target_frame', default_value='vicon1', # this value can be passed through as launch... 
            description='Target frame name.'
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'} # broadcast position of vicon1
            ]
        ),
        # Node(
        #     package='learning_tf2_py',
        #     executable='turtle_tf2_broadcaster',
        #     name='broadcaster2',
        #     parameters=[
        #         {'turtlename': 'turtle2'} # the actual robot
        #     ]
        # ),
        Node( 
            package='learning_tf2_py',
            executable='turtle_tf2_target_publisher',
            name='publisher',
            parameters=[
                {'turtlename': 'turtle1'} #publish vicon1 frame to set velocity
            ]
        ),
        Node(
            package='learning_tf2_py',
            executable='turtle_tf2_listener',
            name='listener',
            parameters=[
                {'target_frame': 'turtle1'} # move turtle1 to vicon1
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '0', '0', 'world', 'odom']
        ),
    ])