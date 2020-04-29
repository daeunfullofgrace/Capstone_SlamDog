#!/usr/bin/env python

from qrcode.srv import *
import rospy
import qrcode_scanner

def button_clicked(req):
   print "Returning [%s + %s = %s]" %(req.a, req.b, (req.a+req.b))
   qrcode_scanner.is_btn_clicked = 1

def server():
   # rospy.init_node('qr_code_scanner', anonymous=False)
   s = rospy.Service('qrcode_data', CustomQrCodeSrv, button_clicked)
   print "QrCode Data Server"
   
   rospy.spin()

if __name__=="__main__":
    server()