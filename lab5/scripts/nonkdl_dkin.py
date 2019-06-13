#! /usr/bin/python

import rospy
import json
import os
from tf.transformations import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)


def correct(data):
    if data.position[0] < rest['i1'][0] or data.position[0] > rest['i1'][1]:
        return False

    if data.position[1] < rest['i2'][0] or data.position[1] > rest['i2'][1]:
        return False

    if data.position[2] < rest['i3'][0] or data.position[2] > rest['i3'][1]:
        return False

    return True


def forward_kinematics(data):
    if not correct(data):
        rospy.logerr('Incorrect position! ' + str(data))
        return

    a, d, al, th = params['i1']
    al, a, d, th = float(al), float(a), float(d), float(th)
    tz = translation_matrix((0, 0, d))
    rz = rotation_matrix(data.position[0], zaxis)
    tx = translation_matrix((a, 0, 0))
    rx = rotation_matrix(al, xaxis)
    T1 = concatenate_matrices(rx, tx, rz, tz)

    a, d, al, th = params['i2']
    al, a, d, th = float(al), float(a), float(d), float(th)
    tz = translation_matrix((0, 0, d))
    rz = rotation_matrix(data.position[1], zaxis)
    tx = translation_matrix((a, 0, 0))
    rx = rotation_matrix(al, xaxis)
    T2 = concatenate_matrices(rx, tx, rz, tz)

    a, d, al, th = params['i3']
    al, a, d, th = float(al), float(a), float(d), float(th)
    tz = translation_matrix((0, 0, data.position[2]))
    rz = rotation_matrix(th, zaxis)
    tx = translation_matrix((a, 0, 0))
    rx = rotation_matrix(al, xaxis)
    T3 = concatenate_matrices(rx, tx, rz, tz)

    Tk = concatenate_matrices(T1, T2, T3)
    x, y, z = translation_from_matrix(Tk)
    qx, qy, qz, qw = quaternion_from_matrix(Tk)

    pose = PoseStamped()
    pose.header.frame_id = 'base_link'
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    pub.publish(pose)

    marker = Marker()
    marker.header.frame_id = 'base_link'
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.pose.orientation.w = 1

    time = rospy.Duration()
    marker.lifetime = time
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = qx;
    marker.pose.orientation.y = qy;
    marker.pose.orientation.z = qz;
    marker.pose.orientation.w = qw;
    marker.color.a = 0.7
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('NONKDL_KIN', anonymous=True)

    pub = rospy.Publisher('ambrozy', PoseStamped, queue_size=10)
    marker_pub = rospy.Publisher('nkdl_visualization', Marker, queue_size=100)

    rospy.Subscriber('joint_states', JointState, forward_kinematics)

    params = {}
    print os.path.dirname(os.path.realpath(__file__))
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../dh.json', 'r') as file:
        params = json.loads(file.read())

    rest = {}
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../rest.json', 'r') as file:
        rest = json.loads(file.read())

    rospy.spin()
