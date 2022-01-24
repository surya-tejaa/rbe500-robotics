#!/usr/bin/env python3

import sys
import rospy
from jacobian_ga3.srv import *

def jacobian_client(x, y, z):
    rospy.wait_for_service('eejoint')
    try:
        eejoint = rospy.ServiceProxy('eejoint', EEJoint)
        resp = eejoint(x,y,z)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y z]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s %s %s"%(x, y, z))
    print("%s"%(jacobian_client(x, y, z)))