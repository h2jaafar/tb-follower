import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from rosgraph_msgs.msg import Clock


from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from turtlesim.srv import Spawn
import time

class FrameListener(Node):

    def __init__(self):
        super().__init__('turtlebot_tf2_target_listener')

        # Declare and acquire `target_frame` parameter
        self.target_frame = 'target'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a client to spawn a turtle

        time.sleep(3) # sleep until everything is spawned
        self.turtle_spawning_service_ready = True
        # if the turtle was successfully spawned
        self.turtle_spawned = True
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)

        # clock publishes at 10Hz btw from gazebo...so callback is called 10 times a second
        self.subscription = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback_lis,
            1)
        self.subscription  # prevent unused variable warning
        # Create turtle2 velocity publisher
        self._loop_rate = self.create_rate(10, self.get_clock())

        # Call on_timer function every second
        # self.timer = self.create_timer(1.0, self.on_timer)

    def clock_callback_lis(self, msg):
        # Store frame names in variables that will be used to
        # compute transformations
        # rate = self.create_rate(10)


        from_frame_rel = self.target_frame
        to_frame_rel = 'base_link'
        # Look up for the transformation between target_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach target_frame
        # try:
        now = msg.clock
        # now.sec = now.sec - 2
        try:
            # now = self.get_clock().now()
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                now,
                timeout=rclpy.duration.Duration(seconds=3.0))
            self.get_logger().info('transformed success')
            
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info('transform not ready')
            # self.get_logger().info()
            self.get_logger().info(
                f'not ready to transform {to_frame_rel} to {from_frame_rel}')
            # raise
            # self._loop_rate.sleep()
            return
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        msg = Twist()
        scale_rotation_rate = 1.0
        msg.angular.z = scale_rotation_rate * math.atan2(
            t.transform.translation.y,
            t.transform.translation.x)

        scale_forward_speed = 0.5
        msg.linear.x = scale_forward_speed * math.sqrt(
            t.transform.translation.x ** 2 +
            t.transform.translation.y ** 2)

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()