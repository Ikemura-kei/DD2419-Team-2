#!/usr/bin/env python

import math

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

from geometry_msgs.msg import TransformStamped
from robp_interfaces.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class Odometry(Node):
    # --------------------------------
    # -- robot kinematic parameters --
    # --------------------------------
    WHEEL_RADIUS = 9.8 * 0.01 / 2 # m
    WHEEL_SEPARATION = 31 * 0.01 # m

    def __init__(self):
        super().__init__('odometry')

        # Initialize the transform broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)

        # Initialize the path publisher
        self._path_pub = self.create_publisher(Path, 'path', 10)
        # Store the path here
        self._path = Path()
        
        self.tf_broadcast_timer = self.create_timer(0.01, self.timer_callback)
    
        # Subscribe to encoder topic and call callback function on each recieved message
        self.create_subscription(
            Encoders, '/motor/encoders', self.encoder_callback, 10)

        # 2D pose
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self.stamp = None
        
        self.last_time = None
        
        self.last_tf_broadcast_time = None
        
    def timer_callback(self):
        if self.stamp is None:
            return
        
        self.broadcast_transform(self.get_clock().now().to_msg(), self._x, self._y, self._yaw)
        self.publish_path(self.get_clock().now().to_msg(), self._x, self._y, self._yaw)
        
        if self.last_tf_broadcast_time is None:
            self.last_tf_broadcast_time = self.get_clock().now()
            return
        
        dt = (self.get_clock().now() - self.last_tf_broadcast_time).nanoseconds / 1e9
        # print('dt: %f' % dt)
        self.last_tf_broadcast_time = self.get_clock().now()

    def encoder_callback(self, msg: Encoders):
        # The kinematic parameters for the differential configuration
        if self.last_time is None:
            self.last_time = msg.header.stamp
            return
        
        cur_t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        last_t = self.last_time.sec + self.last_time.nanosec / 1e9
        dt = (cur_t - last_t)
        
        if dt == 0:
            return
        
        self.last_time = msg.header.stamp
        
        ticks_per_rev = 48 * 64
        wheel_radius = self.WHEEL_RADIUS
        # if angle too big: shrink base, else: grow base. Best value: 0.312
        base = self.WHEEL_SEPARATION

        # Ticks since last message
        delta_ticks_left = msg.delta_encoder_left
        delta_ticks_right = msg.delta_encoder_right

        v = (wheel_radius/2)*(((delta_ticks_left/ticks_per_rev * (2*math.pi)) + (delta_ticks_right/ticks_per_rev * (2*math.pi)))/dt)
        w = (wheel_radius/base)*(((delta_ticks_right/ticks_per_rev * (2*math.pi)) - (delta_ticks_left/ticks_per_rev * (2*math.pi)))/dt)
        D = v * dt

        self._x = self._x + D * math.cos(self._yaw)
        self._y = self._y + D * math.sin(self._yaw)
        self._yaw = self._yaw + w * dt
                
        stamp = msg.header.stamp
        self.stamp = stamp

        # self.get_logger().info('Odometry: x: %f, y: %f, yaw: %f, stamp sec: %f, stamp nanosec: %f' % (self._x, self._y, self._yaw, stamp.sec, stamp.nanosec))
        
    def broadcast_transform(self, stamp, x, y, yaw):
        """Takes a 2D pose and broadcasts it as a ROS transform.
        Broadcasts a 3D transform with z, roll, and pitch all zero. 
        The transform is stamped with the current time and is between the frames 'odom' -> 'base_link'.
        Keyword arguments:
        stamp -- timestamp of the transform
        x -- x coordinate of the 2D pose
        y -- y coordinate of the 2D pose
        yaw -- yaw of the 2D pose (in radians)
        """

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # The robot only exists in 2D, thus we set x and y translation
        # coordinates and set the z coordinate to 0
        t.transform.translation.x = x   
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        # For the same reason, the robot can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0.0, 0.0, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self._tf_broadcaster.sendTransform(t)

    def publish_path(self, stamp, x, y, yaw):
        """Takes a 2D pose appends it to the path and publishes the whole path.
        Keyword arguments:
        stamp -- timestamp of the transform
        x -- x coordinate of the 2D pose
        y -- y coordinate of the 2D pose
        yaw -- yaw of the 2D pose (in radians)
        """

        self._path.header.stamp = stamp
        self._path.header.frame_id = 'odom'

        pose = PoseStamped()
        pose.header = self._path.header

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.01  # 1 cm up so it will be above ground level

        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self._path.poses.append(pose)

        #self._path_pub.publish(self._path)


def main():
    rclpy.init()
    node = Odometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()