#!/usr/bin/env python3

import rospy
import math
import numpy as np
from jacobian_ga3.srv import JointEE, JointEEResponse, EEJoint, EEJointResponse

def joint_ee(req):
    #solve jacobian here
    l1 = 1
    l2 = 1
    theta1 = 45
    theta2 = 45

    q1_ = req.q1
    q2_ = req.q2
    d3_ = req.d3

    s1 = math.sin(theta1)
    c1 = math.cos(theta1)
    s12 = (math.cos(theta1)*math.sin(theta2)) + (math.sin(theta1)*math.cos(theta2))
    c12 = (math.cos(theta1)*math.cos(theta2)) - (math.sin(theta1)*math.sin(theta2))

    r11 = (-l1*s1) - (l2*s12)
    r12 = -l2*s12
    r13 = 0

    r21 = (l1*c1)+(l2*c12)
    r22 = l2*c12
    r23 = 0

    matrixJ = np.array([[r11, r12, r13], [r21, r22, r23], [0, 0, -1], [0, 0, 0], [0, 0, 0], [1, 1, 0]])
    joint_vel = np.array([[q1_], [q2_], [d3_]])
    Xdot = np.dot(matrixJ, joint_vel)
    print("x = ", Xdot[0])
    print("y = ", Xdot[1])
    print("z = ", Xdot[2])

    # print(q1_)
    # print(q2_)
    # print(d3_)

    return JointEEResponse(x=(Xdot[0]), y=(Xdot[1]), z=(Xdot[2]))

def ee_joint(req):
        #solve jacobian here
    l1 = 1
    l2 = 1
    theta1 = 45
    theta2 = 45

    x_ = req.x
    y_ = req.y
    z_ = req.z

    s1 = math.sin(theta1)
    c1 = math.cos(theta1)
    s12 = (math.cos(theta1)*math.sin(theta2)) + (math.sin(theta1)*math.cos(theta2))
    c12 = (math.cos(theta1)*math.cos(theta2)) - (math.sin(theta1)*math.sin(theta2))

    r11 = (-l1*s1) - (l2*s12)
    r12 = -l2*s12
    r13 = 0

    r21 = (l1*c1)+(l2*c12)
    r22 = l2*c12
    r23 = 0

    matrixJ = np.array([[r11, r12, r13], [r21, r22, r23], [0, 0, -1], [0, 0, 0], [0, 0, 0], [1, 1, 0]])
    ee_vel = np.array([[x_], [y_], [z_]])
    Jinv = np.linalg.inv(matrixJ[0:3, :])
    Xdot = np.dot(Jinv, ee_vel)

    print("q1 = ", Xdot[0])
    print("q2 = ", Xdot[1])
    print("d3 = ", Xdot[2])

    # print(matrixJ)

    # print(x_)
    # print(y_)
    # print(z_)

    return EEJointResponse(q1=(Xdot[0]), q2=( Xdot[1]), d3=(Xdot[2]))

def jacobian_server():
    rospy.init_node('jacobian_server')
    s1 = rospy.Service('jointee', JointEE, joint_ee)
    s2 = rospy.Service('eejoint', EEJoint, ee_joint)
    rospy.spin()

if __name__ == '__main__':
    jacobian_server()