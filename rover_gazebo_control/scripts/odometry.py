#!/usr/bin/env python


import rospy
import math
from math import cos, sin, pi, tan
import tf
from geometry_msgs.msg import Twist, Vector3, TransformStamped, Quaternion, Pose, Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


def rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0

    radius = v / omega

    return math.atan(wheelbase / radius)


def shutdownhook():
    rospy.loginfo(rospy.get_caller_id() + " stop")


class Self_odom():
    def __init__(self):
        self.header = 0
        namespace = rospy.get_namespace()
        self.x_ = 0.0
        self.cmd_x = 0
        self.y_ = 0.0
        self.z = 0
        self.th = 0.
        self.vx = 0.
        self.vy = 0.
        self.vth = 0.
        self.yaw_ = 0
        self.rpm = 0
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_broadcaster1 = tf.TransformBroadcaster()
        self.odom_pub = rospy.Publisher("/odom_fake", Odometry, queue_size=20000)
        self.cmd_subscriber = rospy.Subscriber("/rover/ackermann_cmd", AckermannDriveStamped, self.call_back_cmd)

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        rospy.on_shutdown(shutdownhook)

    def call_back(self, msg):
        # odometry data from hall sensors
        self.rpm = msg.data

    # self.rpm = (msg.data[0] * pi * 0.08) / 60

    def call_back_cmd(self, data):
        self.cmd_x = data.drive.speed
        self.current_time = data.header.stamp
        self.z = data.drive.steering_angle

    def pub_odom(self):
        delta_time = (self.current_time - self.last_time).to_sec()  # seconds

        # basic velocity inputs
        current_speed = self.cmd_x  # m/s from rear wheels
        steering_angle = self.z  # radians

        # compute odometry values from joint angles
        # and get the theta update
        angular_velocity = current_speed * tan(steering_angle) / 0.806
        # rad

        # compute odometry update values
        delta_x = current_speed * cos(self.yaw_)
        delta_y = current_speed * sin(self.yaw_)

        # now update our pose estimate
        self.x_ += delta_x * delta_time
        self.y_ += delta_y * delta_time

        self.yaw_ += angular_velocity * delta_time

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.yaw_)

        # first, we'll publish the transform over tf
        self.tf_broadcaster1.sendTransform(
            (self.x_, self.y_, 0.0),
            odom_quat,
            self.current_time,
            "chassis_footprint",
            "odom"
        )

        # now update our pose estimate
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"
        odom.pose.pose = Pose(Point(self.x_, self.y_, 0.0), Quaternion(*odom_quat))

        # Co variance should be cacluated empirically
        odom.pose.covariance[0] = 0.2  # < x
        odom.pose.covariance[7] = 0.2  # < y
        odom.pose.covariance[35] = 0.4  # < yaw

        # Set Velocity
        odom.child_frame_id = 'chassis_footprint'
        odom.twist.twist = Twist(Vector3(current_speed, 0, 0), Vector3(0, 0, angular_velocity))
        self.odom_pub.publish(odom)
        self.last_time = self.current_time

        #
        # self.tf_config()
        # self.current_time = rospy.Time.now()
        # dt = (self.current_time - self.last_time).to_sec()
        #
        # delta_x = (self.vx * cos(self.th)) * dt - self.vy * sin(self.th)
        # delta_y = (self.vx * sin(self.th)) * dt + self.vy * cos(self.th)
        #
        # delta_th = self.vth * dt
        #
        # self.th = self.th + delta_th
        # yaw = self.th * (180 / pi)
        #
        # self.x = self.x + delta_x
        # self.y = self.y + delta_y
        #
        # self.last_time = self.current_time
        #
        # # publish odom
        #
        # odom = Odometry()
        # odom.header.stamp = self.current_time
        # odom.header.frame_id = "odom"
        # odom.pose.pose.position.x = self.x
        # odom.pose.pose.position.y = self.y
        # odom.pose.pose.position.z = self.z
        #
        # odom.pose.pose.orientation.x = 0
        # odom.pose.pose.orientation.y = 0
        # odom.pose.pose.orientation.z = sin(yaw / 2.0)
        # odom.pose.pose.orientation.w = cos(yaw / 2.0)
        #
        # odom.child_frame_id = "chassis_footprint"
        # odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        #
        # odom.twist.twist = Twist(Vector3(self.vx, 0, 0), Vector3(0, 0, self.vth))
        # # self.tf_broadcaster.sendTransformMessage(transform_odom)
        # self.tf_broadcaster1.sendTransform(
        #     (self.x, self.y, 0.0),
        #     odom_quat,
        #     self.current_time,
        #     "chassis_footprint",
        #     "odom"
        # )
        # self.odom_pub.publish(odom)


if __name__ == '__main__':

    rospy.init_node('odom')
    self_odom = Self_odom()
    while not rospy.is_shutdown():
        self_odom.pub_odom()

        r = rospy.Rate(100)
        r.sleep()
