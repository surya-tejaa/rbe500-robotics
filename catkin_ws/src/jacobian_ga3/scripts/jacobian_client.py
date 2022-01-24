#!/usr/bin/env python3

import sys
import rospy
from jacobian_ga3.srv import *

def jacobian_client(q1, q2, d3):
    rospy.wait_for_service('jointee')
    try:
        jointee = rospy.ServiceProxy('jointee', JointEE)
        resp = jointee(q1,q2,d3)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [q1 q2 d3]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        q1 = float(sys.argv[1])
        q2 = float(sys.argv[2])
        d3 = float(sys.argv[3])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s %s %s"%(q1, q2, d3))
    print("%s"%(jacobian_client(q1, q2, d3)))