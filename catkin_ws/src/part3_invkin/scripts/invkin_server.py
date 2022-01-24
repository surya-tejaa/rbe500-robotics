#!/usr/bin/env python3

import rospy
from part3_invkin.srv import InvKin, InvKinResponse
import math

def inv_kin(req):
    a = 0.7
    l1 = 0.05
    l2 = 0.7
    xsquare = math.pow(req.x, 2)
    ysquare = math.pow(req.y, 2)
    asquare = a**2
    lsquare = l2**2

    theta2_ = math.acos((xsquare + ysquare - asquare - lsquare) / (2*a*l2))
    theta1_ = math.atan2(req.y, req.x) - theta2_
    d3_ = l1 - req.z
    #print joint values
    print(theta1_)
    print(theta2_)
    print(d3_)
    return InvKinResponse(theta1=(theta1_), theta2=(theta2_), d3=(d3_))

def invkin_server():
    rospy.init_node('invkin_server')
    s = rospy.Service('invkin', InvKin, inv_kin)
    rospy.spin()

if __name__ == "__main__":
    invkin_server()