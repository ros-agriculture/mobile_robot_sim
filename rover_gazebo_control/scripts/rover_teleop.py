#!/usr/bin/env python

"""
rover_teleop

Takes the keyboard inputs and publishes the linear and
angular velocity commands.

Published Topics:
    cmd_vel (geometry_msgs/Twist)
"""

import curses
import math

import rospy
from geometry_msgs.msg import Twist


class SetVelocity():
    def __init__(self, velocity, num_steps):
        assert velocity != 0 and num_steps > 0
        self._velocity = velocity
        self._num_steps = num_steps
        if self._num_steps > 1:
            self._step_incr = (self._velocity) / (self._num_steps - 1)
        else:
            # If num_steps is one, we always use the minimum velocity.
            self._step_incr = 0

    def __call__(self, value, step):
        """
        Takes a value in the range [0, 1] and the step and returns the
        velocity (usually m/s or rad/s).
        """
        if step == 0:
            return 0

        assert step > 0 and step <= self._num_steps
        max_value = self._step_incr * (step - 1)
        return value * max_value


class TextWindow():

    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=10):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = lines

    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message):
        if lineno < 0 or lineno >= self._num_lines:
            raise ValueError
        height, width = self._screen.getmaxyx()
        y = (height / self._num_lines) * lineno
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            self._screen.addstr(y, x, text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()


class KeyTeleop():
    _interface = None

    def __init__(self, interface):
        self._interface = interface
        namespace = rospy.get_namespace()
        self._pub_cmd = rospy.Publisher(
            namespace + 'cmd_vel', Twist, queue_size=1)

        self._hz = rospy.get_param('~hz', 400)

        self._num_steps = rospy.get_param('~steps')

        linear_vel = rospy.get_param('~linear_velocity', 1.0)
        self._get_linear = SetVelocity(linear_vel, self._num_steps)

        angular_vel = rospy.get_param('~angular_velocity', 1.2)
        self._get_rotation = SetVelocity(angular_vel, self._num_steps)

    def run(self):
        self._linear = 0
        self._angular = 0

        rate = rospy.Rate(self._hz)
        self._running = True
        try:
            while self._running:
                keycode = self._interface.read_key()
                if keycode:
                    if self._key_pressed(keycode):
                        self._publish()
                else:
                    self._publish()
                    rate.sleep()
        except rospy.ROSInterruptException:
            pass

    def _get_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = self._get_linear(
            math.copysign(1, linear), abs(linear))
        twist.angular.z = self._get_rotation(
            math.copysign(1, angular), abs(angular))
        return twist

    def _key_pressed(self, keycode):
        movement_bindings = {
            curses.KEY_UP:    (1,  0),
            curses.KEY_DOWN:  (-1,  0),
            curses.KEY_LEFT:  (0,  1),
            curses.KEY_RIGHT: (0, -1),
        }
        speed_bindings = {
            ord(' '): (0, 0),
        }
        if keycode in movement_bindings:
            acc = movement_bindings[keycode]
            ok = False
            if acc[0]:
                linear = self._linear + acc[0]
                if abs(linear) <= self._num_steps:
                    self._linear = linear
                    ok = True
            if acc[1]:
                angular = self._angular + acc[1]
                if abs(angular) <= self._num_steps:
                    self._angular = angular
                    ok = True
            if not ok:
                self._interface.beep()
        elif keycode in speed_bindings:
            acc = speed_bindings[keycode]
            # Note: bounds aren't enforced here!
            if acc[0] is not None:
                self._linear = acc[0]
            if acc[1] is not None:
                self._angular = acc[1]

        elif keycode == ord('q'):
            self._running = False
            rospy.signal_shutdown('Bye')
        else:
            return False

        return True

    def _publish(self):
        twist = self._get_twist(self._linear, self._angular)

        self._interface.clear()
        self._interface.write_line(2, 'Linear: %f, Angular: %f, Steps: %d' %
                                   (round(twist.linear.x, 3), round(twist.angular.z, 3), self._num_steps))
        # self._interface.write_line(2, 'Linear: %d, Angular: %d' % (self._linear, self._angular))
        self._interface.write_line(
            5, 'Use arrow keys to move, space to stop, q to exit.')
        self._interface.refresh()

        self._pub_cmd.publish(twist)


def main(stdscr):
    rospy.init_node('key_teleop')
    app = KeyTeleop(TextWindow(stdscr))
    app.run()


if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass
