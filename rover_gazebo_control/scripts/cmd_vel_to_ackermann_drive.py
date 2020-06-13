#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de

import math
import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive


def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0

    radius = v / omega
    return math.atan(wheelbase / radius)


def cmd_callback(data):
    global wheelbase
    global ackermann_cmd_topic
    global frame_id
    global pub

    v = data.linear.x
    steering = convert_trans_rot_vel_to_steering_angle(
        v, data.angular.z, 0.6)

    msg = AckermannDrive()
    # msg.header.stamp = rospy.Time.now()
    # msg.header.frame_id = frame_id
    # msg.drive.steering_angle = steering
    # msg.drive.speed = v
    msg.steering_angle = steering
    msg.speed = v

    pub.publish(msg)


if __name__ == '__main__':
    try:
        rospy.init_node('cmd_vel_to_ackermann_drive')
        _wheelbase = 1.0
        namespace = rospy.get_namespace()
        twist_cmd_topic = rospy.get_param(
            '~twist_cmd_topic', namespace + '/cmd_vel')
        ackermann_cmd_topic = rospy.get_param(
            '~ackermann_cmd_topic', namespace + 'ackermann_cmd')
        frame_id = rospy.get_param('~frame_id', 'odom')
        try:
            wheelbase = rospy.get_param(namespace + "wheelbase")
            if wheelbase <= 0.0:
                raise ValueError()
        except Exception:
            rospy.logwarn("The specified wheelbase value is invalid."
                          "Instead, the default wheelbase value will be used.")
            wheelbase = _wheelbase
        rospy.Rate(100)
        rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=2)
        pub = rospy.Publisher(ackermann_cmd_topic,
                              AckermannDrive, queue_size=2)

        rospy.loginfo(
            "Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s.\nPublishing to %s.\nFrame id: %s, \nWheelbase: %f",
            "/cmd_vel", ackermann_cmd_topic, frame_id, wheelbase)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
