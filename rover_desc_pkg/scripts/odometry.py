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
        self.count = 0
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_broadcaster1 = tf.TransformBroadcaster()
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.cmd_subscriber = rospy.Subscriber("/rover/ackermann_cmd", AckermannDriveStamped, self.call_back_cmd)
        self.pose_gx = 0
        self.pose_gy = 0
        self.pose_ox = 0
        self.pose_oy = 0
        self.x1_ = 0
        self.y1_ = 0
        self.yaw_odom = 0
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        rospy.on_shutdown(shutdownhook)

    def call_back_cmd(self, data):
        self.count = self.count + 1
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

        delta_x1 = current_speed * cos(self.yaw_odom) / 0.082
        delta_y1 = current_speed * sin(self.yaw_odom) / 0.082

        # now update our pose estimate
        self.x_ += delta_x * delta_time
        self.y_ += delta_y * delta_time

        self.x1_ += delta_x1 * delta_time
        self.y1_ += delta_y1 * delta_time

        self.yaw_ += angular_velocity * delta_time

        self.yaw_odom += angular_velocity * delta_time / 0.082

        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.yaw_)
        odom_quat1 = tf.transformations.quaternion_from_euler(0, 0, self.yaw_odom)
        # first, we'll publish the transform over tf

        # self.tf_broadcaster1.sendTransform(
        #     (self.x_, self.y_, 0.0),
        #     odom_quat,
        #     rospy.Time.now(),
        #     "chassis_footprint",
        #     "odom"
        # )

        # now update our pose estimate
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"
        odom.pose.pose = Pose(Point(self.x_, self.y_, 0.0), Quaternion(*odom_quat))

        # Co variance should be cacluated empirically
        odom.pose.covariance[0] = 0.0  # < x
        odom.pose.covariance[7] = 0.0  # < y
        odom.pose.covariance[35] = 0.0  # < yaw

        # Set Velocity
        odom.child_frame_id = 'chassis_footprint'
        odom.twist.twist = Twist(Vector3(current_speed, 0, 0), Vector3(0, 0, angular_velocity))

        self.odom_pub.publish(odom)
        self.last_time = self.current_time


if __name__ == '__main__':

    rospy.init_node('odom')
    self_odom = Self_odom()
    while not rospy.is_shutdown():
        self_odom.pub_odom()

        r = rospy.Rate(100)
        r.sleep()
