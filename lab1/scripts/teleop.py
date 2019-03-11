#!/usr/bin/env python
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def teleop():
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('teleop', anonymous=True)

    forward = rospy.get_param("teleop/forward")
    ret = rospy.get_param("teleop/return")
    rotate_left = rospy.get_param("teleop/rotate_l")
    rotate_right = rospy.get_param("teleop/rotate_r")

    while True:
        ch = getch()
        if ch == '\x03':
            raise KeyboardInterrupt

        changed = False
        cmd = Twist()

        if ch == forward:
            cmd.linear.x = 1.0
            changed = True
        elif ch == ret:
            cmd.linear.x = -1.0
            changed = True
        elif ch == rotate_left:
            cmd.angular.z = 1.0
            changed = True
        elif ch == rotate_right:
            cmd.angular.z = -1.0
            changed = True

        if changed:
            pub.publish(cmd)

        rospy.loginfo(ch)


if __name__ == '__main__':
    try:
        teleop()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
