#!/usr/bin/env python

import sys
import rospy
from qrcode.srv import *

def qrcode_data_client(x,y):
   rospy.wait_for_service('qrcode_data')
   try:
      request = rospy.ServiceProxy('qrcode_data', CustomQrCodeSrv)
      resp1 = request(x,y)
      return resp1.sum
   except rospy.ServiceException, e:
      print "Service call failed: %s" %e

def usage():
   return "%s [x y]" %sys.argv[0]

if __name__=="__main__":
   if len(sys.argv) == 3 :
      x = int(sys.argv[1])
      y = int(sys.argv[2])
   else:
      print usage()
      sys.exit(1)

   print "Requesting %s + %s" %(x,y)
   print "%s + %s = %s" %(x, y, qrcode_data_client(x,y))