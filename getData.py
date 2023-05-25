#!/usr/bin/env python
 
import rospy
from xela_server.srv import XelaSensorXYZ
from xela_server.msg import xServerMsg
 
def callback(xeladata):
  if xeladata.sensor == 2:
    print(xeladata.sensor)
    print(xeladata.points[15])
  
 
rospy.init_node('xela_use_rostopic')
rospy.Subscriber('/xServTopic', xServerMsg, callback)
 

rospy.spin()