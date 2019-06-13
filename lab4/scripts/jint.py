#!/usr/bin/env python

import rospy
from lab2.srv import Interpolation
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math


freq = 50


def handle_interpolation(req):
    if req.t <= 0 or not -1.57075 <= req.j1 <= 1.57075 or not -1.57075 <= req.j2 <= 1.57075 or not 0 <= req.j3 <= 0.2:
        return False

    current_pos = rospy.wait_for_message('joint_states', JointState, timeout = 10).position
    new_pos = [req.j1, req.j2, req.j3]

    diff_sum = sum([(new_pos[i] - current_pos[i]) for i in range(0, 3)])

    rate = rospy.Rate(freq)
    j1, j2, j3 = current_pos[0], current_pos[1], current_pos[2]

    frames_number = int(math.ceil(req.t * freq))
    current_time = 0.

    for k in range(0, frames_number + 1):
        computed_joint_state = JointState()
        computed_joint_state.header = Header()
        computed_joint_state.header.stamp = rospy.Time.now()
        computed_joint_state.name = ['base_to_link1', 'link1_to_link2', 'link2_to_link3']

        j1 = compute_int(current_pos[0], new_pos[0], req.t, current_time, req.i)
        j2 = compute_int(current_pos[1], new_pos[1], req.t, current_time, req.i)
        j3 = compute_int(current_pos[2], new_pos[2], req.t, current_time, req.i)

        computed_joint_state.position = [j1, j2, j3]
        computed_joint_state.velocity = []
        computed_joint_state.effort = []
        pub.publish(computed_joint_state)
        current_time = current_time + 1. / freq
        rate.sleep()

    return True


def compute_int(start_j, last_j, time, current_time, i):
    if i == 'tri':
        return compute_tri(start_j, last_j, time, current_time)
    else:
        return compute_const(start_j, last_j, time, current_time)


def compute_const(start_j, last_j, time, current_time):
    return start_j + (float(last_j - start_j) / time) * current_time


def compute_tri(start_j, last_j, time, current_time):
    h = 2. * float(last_j - start_j) / time
    ratio = h / (time / 2.)
    if current_time < time / 2.:
        return start_j + current_time**2 * ratio / 2.
    else:
        return last_j - (time-current_time)**2 * ratio / 2.


if __name__ == "__main__":
    rospy.init_node('int_srv')
    pub = rospy.Publisher('interpolation', JointState, queue_size=10)
    s = rospy.Service('int', Interpolation, handle_interpolation)
    rospy.spin()