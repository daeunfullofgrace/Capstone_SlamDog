#!/usr/bin/env python

from __future__ import print_function
import sys
import rospy
from data_manager.srv import *


def loc_data_client(x):
    rospy.wait_for_service('location_data')
    try:
        request = rospy.ServiceProxy('location_data', dataLoader)
        resp1 = request(x)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed" % e)


def usage():
    return "%s [x]" % sys.argv[0]


if __name__ == "__main__":
    if len(sys.argv) == 2:
        x = sys.argv[1]
    else:
        print(usage())
        sys.exit(1)

    print("Requesting %s" % x)
    print("%s" %(loc_data_client(x)))
