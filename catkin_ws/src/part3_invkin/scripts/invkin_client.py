#!/usr/bin/env python3

import sys
import rospy
from part3_invkin.srv import *

def invkin_client(x, y, z):
    rospy.wait_for_service('invkin')
    try:
        invkin = rospy.ServiceProxy('invkin', InvKin)
        resp = invkin(x,y,z)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y z]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        x = sys.argv[1]
        y = sys.argv[2]
        z = sys.argv[2]
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s %s %s"%(x, y, z))
    print("%s"%(invkin_client(x, y, z)))