import math

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from turtlesim.msg import Pose
from rosgraph_msgs.msg import Clock


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class FramePublisher(Node):

    def __init__(self):
        super().__init__('turtlebot_tf2_target_broadcaster')

        
        # iteration from 0-360
        self.iter = 0;
        
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.positions = self.cirlceArrayCreater(radius = 5, stepSize=1/1000, origin=[0,0])

        # clock publishes at 10Hz btw from gazebo...so callback is called 10 times a second
        self.subscription = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            1)
        self.subscription  # prevent unused variable warning

    def clock_callback(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        # t.header.stamp = self.get_clock().now().to_msg()
        t.header.stamp = msg.clock

        t.header.frame_id = 'world'
        t.child_frame_id = 'target'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = self.positions[self.iter][0]
        t.transform.translation.y = self.positions[self.iter][1]
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        theta = math.pi/2 + math.atan2(self.positions[self.iter][1], self.positions[self.iter][0])
        q = quaternion_from_euler(0, 0, theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        if self.iter < 3600:
            self.iter = self.iter + 1
        else:
            self.iter = 0

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
    
        
    def cirlceArrayCreater(self, radius, stepSize, origin):
        a = origin[0]
        b = origin[1]
        positions = []
        t = 0
        while t < 2 * math.pi:
            positions.append((radius * math.cos(t) + a, radius * math.sin(t) + b))
            t += stepSize
        return positions

def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()