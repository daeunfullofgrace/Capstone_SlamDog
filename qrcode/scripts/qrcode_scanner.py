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


class QRcodeScanner:
    scanned_data = []
    destination_odem = [[null for col in range(6)] for row in range(10)]

    scanned_data_index = 0

    def __init__(self):

        rospy.loginfo("start!")

        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        img_topic = "/camera/rgb/image_raw"
        rospy.Subscriber(img_topic, Image, self.callback)

        save_topic = "/ros_tutorial_msg"
        rospy.Subscriber(save_topic, Int32, self.write_file)

        while not rospy.is_shutdown():
            self.callback
            self.write_file

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

    def write_file(self, data):
        print(data)

        output1 = QRcodeScanner.scanned_data
        output2 = QRcodeScanner.destination_odem

        f = open("qr_code_data.txt", 'w')

        for i in range(0, len(output1)):
            data = "%" + str(output1[i]) + "\n"
            f.write(data)

            for j in range(0, 6):
                data = str(output2[i][j]) + "\n"
                f.write(data)

        print("saved!")
        f.close()

    def qrcode_decode(self, img):

        if self.scanned_data_index == 0:
            rospy.Subscriber('odom', Odometry, self.get_odom)

        if self.image_received:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            decoded = pyzbar.decode(gray)

            for d in decoded:

                barcode_data = d.data.decode("utf-8")
                text = barcode_data

                QRcodeScanner.scanned_data.append(text)
                QRcodeScanner.scanned_data = list(set(QRcodeScanner.scanned_data))

                if self.scanned_data_index != len(QRcodeScanner.scanned_data):
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

            QRcodeScanner.destination_odem[self.scanned_data_index][0] = round(pos_x, 2)
            QRcodeScanner.destination_odem[self.scanned_data_index][1] = round(pos_y, 2)
            QRcodeScanner.destination_odem[self.scanned_data_index][2] = round(quat_x, 2)
            QRcodeScanner.destination_odem[self.scanned_data_index][3] = round(quat_y, 2)
            QRcodeScanner.destination_odem[self.scanned_data_index][4] = round(quat_z, 2)
            QRcodeScanner.destination_odem[self.scanned_data_index][5] = round(quat_w, 2)

        except():
            pass

    def print_in_list(self):

        print("index : ", self.scanned_data_index)

        for i in range(0, len(QRcodeScanner.scanned_data)):
            if QRcodeScanner.scanned_data is not None:
                print(QRcodeScanner.scanned_data[i], end="  ")

        print()

        for i in range(0, len(QRcodeScanner.destination_odem)):
            if QRcodeScanner.destination_odem is not None:
                print(QRcodeScanner.destination_odem[i], end=" ")

        print()


if __name__ == '__main__':

    rospy.init_node('qrcode_scanner', anonymous=True)
    scanner = QRcodeScanner()
