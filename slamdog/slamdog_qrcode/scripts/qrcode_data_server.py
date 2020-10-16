#!/usr/bin/env python
import os

from slamdog_qrcode.srv import *
import rospy
import qrcode_scanner

def button_clicked(req):
   qrcode_scanner.btn_clicked = 1
   print "Returning [%s + %s = %s]" %(req.a, req.b, (req.a+req.b))

   return CustomQrCodeSrvResponse(req.a + req.b)

def server():
   # rospy.init_node('qr_code_scanner', anonymous=False)
   s = rospy.Service('qrcode_data', CustomQrCodeSrv, button_clicked)
   print "QrCode Data Server"
   rospy.spin()

if __name__=="__main__":
    server()
