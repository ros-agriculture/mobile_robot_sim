#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, TransformStamped, Quaternion, Pose, Point


class Ground_Truth:
    def __init__(self):
        self.pose_gx = 0
        self.pose_gy = 0
        self.pose_ox = 0
        self.pose_oy = 0
        self.ose_gx = 0
        self.ose_gz = 0
        self.ose_oz = 0
        self.ose_oy = 0
        self.cmd_subscriber = rospy.Subscriber("groundtruth", Odometry, self.call_back_odom)
        self.cmd_subscriber = rospy.Subscriber("odom", Odometry, self.call_back_groundtruth)

    def call_back_odom(self, odom):
        self.pose_ox = odom.pose.pose.position.x
        self.pose_oy = odom.pose.pose.position.y
        self.ose_oz = odom.pose.pose.orientation.z

    def call_back_groundtruth(self, odom):
        self.pose_gx = odom.pose.pose.position.x
        self.pose_gy = odom.pose.pose.position.y
        self.ose_gz = odom.pose.pose.orientation.z

    def print_difference(self):
        x_diff = self.pose_ox - self.pose_gx
        y_diff = self.pose_oy - self.pose_gy
        z_oriention = self.ose_gz - self.ose_oz
        print("x difference " + str(x_diff) + "   " + "y difference " + str(y_diff))
        print("x difference " + str(x_diff) + "   " + "y difference " + str(y_diff))


if __name__ == '__main__':

    rospy.init_node('compare')
    self_compare = Ground_Truth()
    while not rospy.is_shutdown():
        r = rospy.Rate(100)
        self_compare.print_difference()

        r.sleep()
