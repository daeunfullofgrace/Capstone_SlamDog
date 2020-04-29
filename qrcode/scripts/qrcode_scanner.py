#!/usr/bin/env python

from __future__ import print_function

import rospy
import cv2
import qrcode_data_server
from nav_msgs.msg import Odometry
import pyzbar.pyzbar as pyzbar
from pyasn1.compat.octets import null
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

is_btn_clicked = 0


def is_btn_clicked():
    pass


def write_file(input1, input2):
    print("file save")
    f = open("qr_code_data.txt", 'w')

    for i in range(0, len(input1)):
        data = str(input1[i]) + "\n"
        f.write(data)

        for j in range(0, 6):
            data = str(input2[i][j]) + "\n"

            f.write(data)

    print("saved!")
    f.close()


class QRcodeScanner:
    scanned_data = []
    destination_odem = [[null for col in range(6)] for row in range(10)]

    scanned_data_index = 0

    def __init__(self):

        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        img_topic = "/camera/rgb/image_raw"
        rospy.Subscriber(img_topic, Image, self.callback)

        rospy.loginfo("start")

        while True:
            self.callback
            qrcode_data_server.server()
            #
            # if is_btn_clicked == 1:
            #     try:
            #         write_file(self.scanned_data, self.destination_odem)
            #     finally:
            #         break

            k = input()

            if k == 's':
                write_file(self.scanned_data, self.destination_odem)

    def callback(self, data):
        # print(is_btn_clicked)
        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image

        self.qrcode_decode(cv_image)

    def qrcode_decode(self, img):

        if self.scanned_data_index == 0:
            rospy.Subscriber('odom', Odometry, self.get_odom)

        if self.image_received:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            decoded = pyzbar.decode(gray)

            for d in decoded:

                barcode_data = d.data.decode("utf-8")
                text = barcode_data

                self.scanned_data.append(text)
                self.scanned_data = list(set(self.scanned_data))

                if self.scanned_data_index != len(self.scanned_data):
                    rospy.Subscriber('odom', Odometry, self.get_odom)
                    self.print_in_list()
                    self.scanned_data_index += 1

    def get_odom(self, msg):

        try:
            pos_x = msg.pose.pose.position.x
            pos_y = msg.pose.pose.position.y

            quat_x = msg.pose.pose.orientation.x
            quat_y = msg.pose.pose.orientation.y
            quat_z = msg.pose.pose.orientation.z
            quat_w = msg.pose.pose.orientation.w

            self.destination_odem[self.scanned_data_index][0] = round(pos_x, 2)
            self.destination_odem[self.scanned_data_index][1] = round(pos_y, 2)
            self.destination_odem[self.scanned_data_index][2] = round(quat_x, 2)
            self.destination_odem[self.scanned_data_index][3] = round(quat_y, 2)
            self.destination_odem[self.scanned_data_index][4] = round(quat_z, 2)
            self.destination_odem[self.scanned_data_index][5] = round(quat_w, 2)

        except():
            pass

    def print_in_list(self):

        print("index : ", self.scanned_data_index)

        for i in range(0, len(self.scanned_data)):
            if self.scanned_data is not None:
                print(self.scanned_data[i], end="  ")

        print()

        for i in range(0, len(self.destination_odem)):
            if self.destination_odem is not None:
                print(self.destination_odem[i], end=" ")

        print()


if __name__ == '__main__':
    # Initialize
    rospy.init_node('qr_code_scanner', anonymous=False)
    camera = QRcodeScanner()
    rospy.sleep(1)
