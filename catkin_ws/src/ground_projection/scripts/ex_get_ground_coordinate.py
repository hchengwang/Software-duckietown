#!/usr/bin/env python
import sys
import rospy
# from duckietown_msgs.srv import EstimateHomography, GetGroundCoord
from ground_projection.srv import EstimateHomography, GetGroundCoord
from duckietown_msgs.msg import Pixel, GroundCoord



# from sensor_msgs.msg import CameraInfo, Image
import numpy as np
# import rospkg
import IPython

def call_service_get_ground_coordinate(req):
  rospy.wait_for_service("/get_ground_coordinate")

  try:
    get_ground_coord = rospy.ServiceProxy('/get_ground_coordinate', GetGroundCoord)
    resp = get_ground_coord(req)
    return resp
  except rospy.ServiceException, e:
    print "Service call failed: %s" % e

if __name__ == "__main__":
  rospy.init_node("ex_get_ground_coordinate")

  uv = Pixel()
  uv.u = 100
  uv.v = 200

  res = call_service_get_ground_coordinate(uv)

  print "response: (%f, %f)" % (res.xy.x, res.xy.y)
