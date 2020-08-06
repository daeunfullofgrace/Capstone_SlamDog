#!/usr/bin/env python

from __future__ import print_function
from data_manager.srv import *
import rospy


def get_loc(req):
    print("Returning %s" % (req.a + " from client"))
    return dataLoaderResponse(req.a + " from client")


def loc_data_server():
    rospy.init_node('data_loader_service', anonymous=False)
    s = rospy.Service('location_data', dataLoader, get_loc)
    print("Ready to get Locate.")
    rospy.spin()


if __name__ == "__main__":
    loc_data_server()
