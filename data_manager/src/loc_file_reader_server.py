#!/usr/bin/env python

from __future__ import print_function
from data_manager.srv import *
import rospy


def file_reader():
    file = open("/home/daeun/catkin_ws/src/qrcode/data/qr_code_data (copy).txt", "r")
    loc_list = ''

    while True:
        line = file.readline()

        if '%' in line:
            loc_list += line

        if not line:
            break
    file.close()
    return loc_list


def get_loc(req):
    loc_list = file_reader()
    print("Returning\n%s" % loc_list)
    return loc_list


def file_reader_server():
    rospy.init_node('file_reader_service', anonymous=False)
    s = rospy.Service('location_data', dataLoader, get_loc)
    print("Ready to get Locate List.")
    rospy.spin()


if __name__ == "__main__":
    file_reader_server()
