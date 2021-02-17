#!/usr/bin/env python

from __future__ import print_function
from slamdog_srv_navi.srv import *
import rospy
import os


def initialization(req):
    print("Returning %s" % (req.a + " from client"))
    os.system("rosrun slamdog_navi pose_initialization")
    os.system("rosrun slamdog_srv_navi send_loc_data_server")
    return dataLoaderResponse(req.a + " from client")


def position_initialization_server():
    rospy.init_node('pose_initialization_server', anonymous=False)
    s = rospy.Service('pose_initialization', dataLoader, initialization)
    print("Ready to Get Client.")
    rospy.spin()


if __name__ == "__main__":
    position_initialization_server()
