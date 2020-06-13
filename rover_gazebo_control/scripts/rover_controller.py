#!/usr/bin/env python

"""
rover_controller

Control the wheels of a rover with Ackermann steering.

Subscribed Topics:
    ackermann_cmd (ackermann_msgs/AckermannDrive)
        Ackermann command. It contains the vehicle's desired speed and steering angle.

Published Topics:
    <front right steering controller name>/command (std_msgs/Float64)
        Command for the right steering controller.
    <front left steering controller name>/command (std_msgs/Float64)
        Command for the left steering controller.
    <front right wheel controller name>/command (std_msgs/Float64)
        Command for the front right wheel controller.
    <front left wheel controller name>/command (std_msgs/Float64)
        Command for the front left wheel controller.
    <rear right wheel controller name>/command (std_msgs/Float64)
        Command for the rear right wheel controller.
    <rear left wheel controller name>/command (std_msgs/Float64)
        Command for the rear left wheel controller.

Services Called:
    controller_manager/list_controllers (controller_manager_msgs/ListControllers)
        List the states of the controllers.

Parameters:
    ~ front_right_wheel/steering_link_name (string, default: front_right_steer)
    ~ front_left_wheel/steering_link_name (string, default: front_left_steer)
        Names of links that have origins coincident with the origins of the
        right and left steering joints, respectively. The steering links are
        used to compute the distance between the steering joints, as well as
        the vehicle's wheelbase.

    ~ front_right_wheel/steering_controller_name (string, default: front_right_steering_ctrlr)
    ~ front_left_wheel/steering_controller_name (string, default: front_left_steering_ctrlr)
        Steering controller names.

    ~ rear_right_wheel/link_name (string, default: rear_right_wheel)
    ~ rear_left_wheel/link_name (string, default: rear_left_wheel)
        Names of the links that have origins coincident with the centers of the
        right and left wheels, respectively. The rear wheel links are used to
        compute the vehicle's wheelbase.

    ~ front_right_wheel/diameter (double, default: 1.0)
    ~ front_left_wheel/diameter (double, default: 1.0)
    ~ rear_right_wheel/diameter (double, default: 1.0)
    ~ rear_left_wheel/diameter (double, default: 1.0)
        Wheel diameters. Each diameter must be greater than zero. Unit: meter.

    ~ cmd_timeout (double, default: 0.5)
        If ~cmd_timeout is greater than zero and this node does not receive a
        command for more than ~cmd_timeout seconds, vehicle motion is paused
        until a command is received. If ~cmd_timeout is less than or equal to
        zero, the command timeout is disabled.

    ~ publishing_frequency (double, default: 30.0)
        Joint command publishing frequency. It must be greater than zero.
        Unit: hertz.

Required TF transforms:
    <~front_right_wheel/steering_link_name> to <~rear_right_wheel/link_name>
        Specifies the position of the front right wheel's steering link in the
        rear right wheel's frame.
    <~front_left_wheel/steering_link_name> to <~rear_right_wheel/link_name>
        Specifies the position of the front left wheel's steering link in the
        rear right wheel's frame.
    <~rear_left_wheel/link_name> to <~rear_right_wheel/link_name>
        Specifies the position of the rear left wheel in the rear right wheel's
        frame.
"""

import math
import numpy
import threading

from math import pi

import rospy
import tf

from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float64
from controller_manager_msgs.srv import ListControllers


class _RoverCtrlr:
    """
    Rover Controller

    An object of class _RoverCtrlr is a node that controls the wheels of a
    rover with Ackermann steering.
    """

    def __init__(self):
        # Parameters
        # Wheels
        (front_left_steer_link_name, front_left_steer_ctrlr_name,
         front_left_wheel_ctrlr_name, self._front_left_inv_circ) = \
            self._get_front_wheel_params("left")
        (front_right_steer_link_name, front_right_steer_ctrlr_name,
         front_right_wheel_ctrlr_name, self._front_right_inv_circ) = \
            self._get_front_wheel_params("right")
        (rear_left_link_name, rear_left_wheel_ctrlr_name,
         self._rear_left_inv_circ) = \
            self._get_rear_wheel_params("left")
        (self._rear_right_link_name, rear_right_wheel_ctrlr_name,
         self._rear_right_inv_circ) = \
            self._get_rear_wheel_params("right")

        list_ctrlrs = rospy.ServiceProxy(
            "controller_manager/list_controllers", ListControllers)
        list_ctrlrs.wait_for_service()

        # Command Timeout
        try:
            self._cmd_timeout = float(rospy.get_param("~cmd_timeout"))
        except Exception:
            rospy.logwarn("The specified command timeout value is invalid."
                          "Instead, the default timeout values will be used.")
            self._cmd_timeout = self._DEF_CMD_TIMEOUT

        # Publishing frequency
        try:
            pub_freq = float(rospy.get_param("~publish_frequency"))
            if pub_freq <= 0.0:
                raise ValueError()
        except Exception:
            rospy.logwarn("The specified publishing frequency is invalid."
                          "Instead, the default frequency will be used.")
            pub_freq = self._DEF_PUB_FREQ
        self._sleep_timer = rospy.Rate(100)

        # _last_cmd_time is the time at which the most recent Ackermann
        # driving command was received.
        self._last_cmd_time = rospy.get_time()

        # _ackermann_cmd_lock is used to control access to _steer_ang,
        # _steer_ang_vel, _speed, and _accel.
        self._ackermann_cmd_lock = threading.Lock()
        self._steer_ang = 0.0      # Steering angle
        self._steer_ang_vel = 0.0  # Steering angle velocity
        self._speed = 0.0
        self._accel = 0.0          # Acceleration

        self._last_steer_ang = 0.0  # Last steering angle
        self._left_theta = 0.0      # Left steering joint angle
        self._right_theta = 0.0     # Right steering joint angle

        self._last_speed = 0.0
        self._last_accel_limit = 0.0  # Last acceleration limit
        # Wheel angular velocities
        self._front_right_ang_vel = 0.0
        self._front_left_ang_vel = 0.0
        self._rear_right_ang_vel = 0.0
        self._rear_left_ang_vel = 0.0

        # _joint_dist_div_2 is the distance between the steering joints,
        # divided by two.
        tfl = tf.TransformListener()
        ls_pos = self._get_link_pos(tfl, front_left_steer_link_name)
        rs_pos = self._get_link_pos(tfl, front_right_steer_link_name)
        self._joint_dist_div_2 = numpy.linalg.norm(ls_pos - rs_pos) / 2
        lrw_pos = self._get_link_pos(tfl, rear_left_link_name)
        rrw_pos = numpy.array([0.0] * 3)
        front_cent_pos = (ls_pos + rs_pos) / 2        # Front center position
        rear_cent_pos = (lrw_pos + rrw_pos) / 2     # Rear center position
        self._wheelbase = numpy.linalg.norm(front_cent_pos - rear_cent_pos)
        self._inv_wheelbase = 1 / self._wheelbase   # Inverse of _wheelbase
        self._wheelbase_sqr = self._wheelbase ** 2

        # Set wheelbase as a parameter.
        rospy.set_param("wheelbase", float(self._wheelbase))

        # Publishers
        self._front_left_steer_cmd_pub = \
            _create_cmd_pub(list_ctrlrs, front_left_steer_ctrlr_name)
        self._front_right_steer_cmd_pub = \
            _create_cmd_pub(list_ctrlrs, front_right_steer_ctrlr_name)

        self._front_left_wheel_cmd_pub = \
            _create_wheel_cmd_pub(list_ctrlrs, front_left_wheel_ctrlr_name)
        self._front_right_wheel_cmd_pub = \
            _create_wheel_cmd_pub(list_ctrlrs, front_right_wheel_ctrlr_name)
        self._rear_left_wheel_cmd_pub = \
            _create_wheel_cmd_pub(list_ctrlrs, rear_left_wheel_ctrlr_name)
        self._rear_right_wheel_cmd_pub = \
            _create_wheel_cmd_pub(list_ctrlrs, rear_right_wheel_ctrlr_name)

        # Subscriber
        self._ackermann_cmd_sub = \
            rospy.Subscriber("ackermann_cmd", AckermannDrive,
                             self.ackermann_cmd_cb, queue_size=1)

    def spin(self):
        """Control the rover."""
        last_time = rospy.get_time()
        try:
            while not rospy.is_shutdown():
                t = rospy.get_time()
                delta_t = t - last_time
                last_time = t

                if (self._cmd_timeout > 0.0 and t - self._last_cmd_time > self._cmd_timeout):
                    # Too much time has elapsed since the last command. Stop the rover.
                    steer_ang_changed, center_y = \
                        self._control_steering(self._last_steer_ang, 0.0, 0.001)
                    self._control_wheels(0.0, 0.0, 0.0, steer_ang_changed, center_y)
                elif delta_t > 0.0:
                    with self._ackermann_cmd_lock:
                        steer_ang = self._steer_ang
                        steer_ang_vel = self._steer_ang_vel
                        speed = self._speed
                        accel = self._accel
                    steer_ang_changed, center_y = \
                        self._control_steering(steer_ang, steer_ang_vel, delta_t)
                    self._control_wheels(speed, accel, delta_t, steer_ang_changed, center_y)

                # Publish the steering and wheel joint commands.
                self._front_left_steer_cmd_pub.publish(self._left_theta)
                self._front_right_steer_cmd_pub.publish(self._right_theta)
                if self._front_left_wheel_cmd_pub:
                    self._front_left_wheel_cmd_pub.publish(self._front_left_ang_vel)
                if self._front_right_wheel_cmd_pub:
                    self._front_right_wheel_cmd_pub.publish(self._front_right_ang_vel)
                if self._rear_left_wheel_cmd_pub:
                    self._rear_left_wheel_cmd_pub.publish(self._rear_left_ang_vel)
                if self._rear_right_wheel_cmd_pub:
                    self._rear_right_wheel_cmd_pub.publish(self._rear_right_ang_vel)

                self._sleep_timer.sleep()
        except rospy.ROSInterruptException as e:
            print(e)

    def ackermann_cmd_cb(self, ackermann_cmd):
        """
        Ackermann driving command cal   lback

        Parameters:
            ackermann_cmd: ackermann_msgs.msg.AckermannDrive
                Ackermann driving command.
        """
        self._last_cmd_time = rospy.get_time()
        with self._ackermann_cmd_lock:

            self._steer_ang = ackermann_cmd.steering_angle
            self._steer_ang_vel = ackermann_cmd.steering_angle_velocity
            self._speed = ackermann_cmd.speed
            self._accel = ackermann_cmd.acceleration

    def _get_front_wheel_params(self, side):
        # Get front wheel parameters. Return a tuple containing the steering
        # link name, steering controller name, wheel controller name (or None),
        # and inverse of the circumference.
        prefix = "~" + "front_" + side + "_wheel/"
        steer_link_name = rospy.get_param(prefix + "steering_link_name",
                                          side + "_steering_link")
        steer_ctrlr_name = rospy.get_param(prefix + "steering_ctrlr_name",
                                           side + "_steering_ctrlr")
        wheel_ctrlr_name, inv_circ = self._get_common_wheel_params(prefix)
        return steer_link_name, steer_ctrlr_name, wheel_ctrlr_name, inv_circ

    def _get_rear_wheel_params(self, side):
        # Get rear wheel parameters. Return a tuple containing the link name,
        # wheel controller name, and inverse of the circumference.
        prefix = "~" + "rear_" + side + "_wheel/"
        link_name = rospy.get_param(prefix + "link_name", side + "_wheel")
        wheel_ctrlr_name, inv_circ = self._get_common_wheel_params(prefix)
        return link_name, wheel_ctrlr_name, inv_circ

    def _get_common_wheel_params(self, prefix):
        # Get parameters used by the front and rear wheels. Return a tuple
        # containing the axle controller name (or None) and the inverse of
        # the circumference.
        wheel_ctrlr_name = rospy.get_param(prefix + "wheel_ctrlr_name",
                                           None)
        try:
            dia = float(rospy.get_param(prefix + "diameter",
                                        self._DEF_WHEEL_DIA))
            if dia <= 0.0:
                raise ValueError()
        except Exception:
            rospy.logwarn("The specified wheel diameter is invalid."
                          "Instead, the default diameter will be used")
            dia = self._DEF_WHEEL_DIA

        return wheel_ctrlr_name, 1 / (pi * dia)

    def _get_link_pos(self, tfl, link):
        # Return the position of the specified link, relative to the right
        # rear wheel link.
        while True:
            try:
                trans, not_used = \
                    tfl.lookupTransform(self._rear_right_link_name,
                                        link, rospy.Time(0))
                return numpy.array(trans)
            except Exception:
                pass

    def _control_steering(self, steer_ang, steer_ang_vel_limit, delta_t):
        # Control tthe steering joints.
        # Compute theta, the virtual front wheel's desired steering angle.
        if steer_ang_vel_limit > 0.0:
            # Limit the steering velocity.
            ang_vel = (steer_ang - self._last_steer_ang) / delta_t
            ang_vel = max(-steer_ang_vel_limit, min(ang_vel, steer_ang_vel_limit))
            theta = self._last_steer_ang + ang_vel * delta_t
        else:
            theta = steer_ang

        # Compute the desired steering angles for the left and right front wheels.
        center_y = self._wheelbase * math.tan((pi / 2) - theta)
        steer_ang_changed = theta != self._last_steer_ang
        if steer_ang_changed:
            self._last_steer_ang = theta
            self._left_theta = _get_steer_ang(math.atan(self._inv_wheelbase *
                                                        (center_y - self._joint_dist_div_2)))
            self._right_theta = _get_steer_ang(math.atan(self._inv_wheelbase *
                                                         (center_y - self._joint_dist_div_2)))

        return steer_ang_changed, center_y

    def _control_wheels(self, speed, accel_limit, delta_t, steer_ang_changed, center_y):
        # Control the wheel joints.
        # Compute rover_speed, the rover's desired speed.
        if accel_limit > 0.0:
            # Limit the vehicle's acceleration.
            self._last_accel_limit = accel_limit
            accel = (speed - self._last_speed) / delta_t
            accel = max(-accel_limit, min(accel, accel_limit))
            rover_speed = self._last_speed + accel * delta_t
        else:
            self._last_accel_limit = accel_limit
            rover_speed = speed

        # Compute the desired angular velocities of the wheels.
        if rover_speed != self._last_speed or steer_ang_changed:
            self._last_speed = rover_speed
            left_dist = center_y - self._joint_dist_div_2
            right_dist = center_y + self._joint_dist_div_2

            # Front
            gain = (2 * pi) * rover_speed / abs(center_y)
            r = math.sqrt(left_dist ** 2 + self._wheelbase_sqr)
            self._front_left_ang_vel = gain * r * self._front_left_inv_circ
            r = math.sqrt(right_dist ** 2 + self._wheelbase_sqr)
            self._front_right_ang_vel = gain * r * self._front_right_inv_circ
            # Rear
            gain = (2 * pi) * rover_speed / center_y
            self._rear_left_ang_vel = \
                gain * left_dist * self._rear_left_inv_circ
            self._rear_right_ang_vel = \
                gain * right_dist * self._rear_right_inv_circ

    _DEF_WHEEL_DIA = 1.0    # Default wheel diameter. Unit: meter.
    _DEF_CMD_TIMEOUT = 0.5  # Default command timeout. Unit: second.
    _DEF_PUB_FREQ = 30.0    # Default publishing frequency. Unit: hertz.


def _wait_for_ctrlr(list_ctrlrs, ctrlr_name):
    # Wait for the specific controller to be in the 'running' state.
    # Commands can be lost if they are published before their controller
    # is running, even if a latched publisher is used.
    while True:
        response = list_ctrlrs()
        for ctrlr in response.controller:
            if ctrlr.name == ctrlr_name:
                if ctrlr.state == "running":
                    return
                rospy.sleep(0.1)
                break


def _create_wheel_cmd_pub(list_ctrlrs, wheel_ctrlr_name):
    # Create a wheel command publisher.
    if not wheel_ctrlr_name:
        return None
    return _create_cmd_pub(list_ctrlrs, wheel_ctrlr_name)


def _create_cmd_pub(list_ctrlrs, ctrlr_name):
    # Create a command publisher.
    _wait_for_ctrlr(list_ctrlrs, ctrlr_name)
    return rospy.Publisher(ctrlr_name + "/command", Float64, queue_size=1)


def _get_steer_ang(phi):
    # Return the desired steering angle for a front wheel.
    if phi >= 0.0:
        return (pi / 2) - phi
    return (-pi / 2) - phi


# main
if __name__ == "__main__":
    rospy.init_node('rover_controller')
    controller = _RoverCtrlr()
    controller.spin()
