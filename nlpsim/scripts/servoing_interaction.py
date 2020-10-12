#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import send_goal
from std_srvs.srv import Empty


class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/detected_human", Image, queue_size = 10)
    self.vel_pub = rospy.Publisher("/robot_1/mobile_base/commands/velocity", Twist, queue_size = 10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/robot_1/camera/rgb/image_raw", Image, self.callback)
    self.depth_sub = rospy.Subscriber("/robot_1/camera/depth/image_raw", Image, self.depth_callback)

    self.centroid = np.zeros(2)
    self.x_centroid = 0.0
    self.y_centroid = 0.0

    self.centroid[0] = self.x_centroid
    self.centroid[1] = self.y_centroid

    self.depval = "None"

    self.height = 480
    self.width = 640

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (self.height, self.width, channels) = cv_image.shape

    # mask = cv2.inRange(cv_image, (30,30,30), (35,35,35))
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    
    mask1 = cv2.inRange(cv_image, (21,10,5), (180,255,255)) # 1. (76, 76, 25)/ (60, 67.1, 29.8)
    mask2 = cv2.inRange(cv_image,  (19,33,28), (180,255,255))# 2. (153, 172, 34)/ (68, 80.2, 67.5)
    mask3 = cv2.inRange(cv_image, (27,27,27), (180,255,255)) # 3. (97, 101, 32)/(63,68.3,39.6)
    # mask4 = cv2.inRange(cv_image, (96,100,30), (98,102,33))

    masks = [mask1, mask2, mask3]

    mask = cv2.bitwise_or(mask1, mask2, mask1)

    cv_image = cv2.bitwise_and(cv_image, cv_image, mask=mask4)
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_HSV2BGR)


    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    _,contours,h = cv2.findContours(gray,1,2)

    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.03*cv2.arcLength(cnt,True),True)
        area = cv2.contourArea(cnt)
        if (len(approx)==5 or len(approx)==4) and area>30000:
            approx = np.squeeze(approx)
            print ("Found human!")
            self.x_centroid = (np.sum(approx[:,0]))/len(approx)
            y_centroid = (np.sum(approx[:,1]))/len(approx)
            cv2.drawContours(cv_image,[cnt],0,255,-1)


    x_des = (self.width/2)
    y_des = (self.height/2)

    if self.x_centroid!=0.0:
        error = (x_des - self.x_centroid)

    else:
        return

    print("Servoing error is {}, and depth value is {}".format(error, self.depval))

    vel_msg = Twist()

    if self.cognitive():
        while (abs(error)<100):
            if (self.depval<1.5):
                print("Sending servoing command!")
                vel_msg.linear.x = 0.5
                vel_msg.angular.z = error*0.05
                self.vel_pub.publish(vel_msg)
                # result = send_goal.movebase_client(0,0,0)
                # pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
                if result:
                    rospy.loginfo("Detected human and stopped!")

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)


  def depth_callback(self, data):

      depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")

      depth_image = np.array(depth_image)

      x_depth = self.centroid[0]
      y_depth = self.centroid[1]

      self.depval = float(depth_image[int(self.centroid[1]), int(self.centroid[0])]) 


  def cognitive(self):
    return True

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
