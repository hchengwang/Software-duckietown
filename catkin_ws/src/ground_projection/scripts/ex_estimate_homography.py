#!/usr/bin/env python
import sys
import rospy
# from duckietown_msgs.srv import EstimateHomography, GetGroundCoord
from ground_projection.srv import EstimateHomography, GetGroundCoord
from duckietown_msgs.msg import Pixel, GroundCoord
from sensor_msgs.msg import CameraInfo, Image
import numpy as np
# import rospkg
import IPython

def call_service_estimate_homography(req):
  rospy.wait_for_service("/estimate_homography")

  try:
    service = rospy.ServiceProxy('/estimate_homography', EstimateHomography)
    resp = service(req)
    return resp
  except rospy.ServiceException, e:
    print "Service call failed: %s" % e

# def image_cb(msg):
#     # print "Delay:%6.3f" % (rospy.Time.now() - msg.header.stamp).to_sec()
#     print "got an image frame..."
#     # rospy.sleep(1.0)


if __name__ == "__main__":
  rospy.init_node("ex_estimate_homography")

  # sub = rospy.Subscriber('/porsche911/rosberrypi_cam/image_raw', Image, image_cb, queue_size=1)

  # rospy.spin()


  # get an image frame
  # img = rospy.wait_for_message("/porsche911/rosberrypi_cam/image_raw", Image)
  img = rospy.wait_for_message("/porsche911/rosberrypi_cam/image_rect", Image)
  print "got an image frame"

  resp = call_service_estimate_homography(img)
  
  print resp


  # uv = Pixel()
  # uv.u = 100
  # uv.v = 200

  # res = call_service_estimate_homography(uv)

  # print "response: (%f, %f)" % (res.xy.x, res.xy.y)
