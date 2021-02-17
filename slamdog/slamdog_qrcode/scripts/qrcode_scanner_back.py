#!/usr/bin/env python

from __future__ import print_function

from std_msgs.msg import Int32

import rospy
import cv2
import qrcode_data_server
from nav_msgs.msg import Odometry
import pyzbar.pyzbar as pyzbar
from pyasn1.compat.octets import null
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class QRcodeScannerBack:
    scanned_data = []
    destination_odem_x = 0;

    scanned_data_index = 0

    def __init__(self):

        rospy.loginfo("start!")

        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        img_topic = "/camera/rgb/image_raw"
        rospy.Subscriber(img_topic, Image, self.callback)

        while not rospy.is_shutdown():
            self.callback

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

                QRcodeScannerBack.scanned_data.append(text)
                QRcodeScannerBack.scanned_data = list(set(QRcodeScannerBack.scanned_data))

                if self.scanned_data_index != len(QRcodeScannerBack.scanned_data):
                    rospy.Subscriber('odom', Odometry, self.get_odom)
                    self.print_in_list()
                    self.scanned_data_index += 1

    def get_odom(self, msg):

        try:
            pos_x = msg.pose.pose.position.x

            destination_odem_x = pos_x

        except():
            pass

    def print_in_list(self):

        print("index : ", self.scanned_data_index)

        for i in range(0, len(QRcodeScannerBack.scanned_data)):
            if QRcodeScannerBack.scanned_data is not None:
                print(QRcodeScannerBack.scanned_data[i], end="  ")

        print()

        if QRcodeScannerBack.destination_odem_x is not None:
            print(QRcodeScannerBack.destination_odem_x, end=" ")

        print()


if __name__ == '__main__':


    rospy.init_node('qrcode_scanner', anonymous=True)
    scanner = QRcodeScannerBack()
