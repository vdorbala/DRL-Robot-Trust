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
from gazebo_connection import GazeboConnection
from std_srvs.srv import Empty

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import *

import matplotlib.pyplot as plt
import time

class cogmod:

  def __init__(self):
    self.image_pub = rospy.Publisher("/detected_human", Image, queue_size = 1)
    # self.vel_pub = rospy.Publisher("/robot_1/mobile_base/commands/velocity", Twist, queue_size = 1)

    self._max_retry = 20
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/robot_1/camera/rgb/image_raw", Image, self.callback, queue_size=1, buff_size=2**24)
    self.depth_sub = rospy.Subscriber("/robot_1/camera/depth/image_raw", Image, self.depth_callback, queue_size=1, buff_size=2**24)
    self.centroid = np.zeros(2)
    self.x_centroid = 0.0
    self.y_centroid = 0.0

    self.centroid[0] = self.x_centroid
    self.centroid[1] = self.y_centroid

    self.depval = "None"

    self.height = 480
    self.width = 640

    self.cogval = True

    self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

  def pauseSim(self):
      rospy.logdebug("PAUSING START")
      rospy.wait_for_service('/gazebo/pause_physics')
      rospy.logdebug("PAUSING service found...")
      paused_done = False
      counter = 0
      while not paused_done and not rospy.is_shutdown():
          if counter < self._max_retry:
              try:
                  rospy.logdebug("PAUSING service calling...")
                  self.pause()
                  paused_done = True
                  rospy.logdebug("PAUSING service callpauseing...DONE")
              except rospy.ServiceException as e:
                  counter += 1
                  rospy.logerr("/gazebo/pause_physics service call failed")
          else:
              error_message = "Maximum retries done"+str(self._max_retry)+", please check Gazebo pause service"
              rospy.logerr(error_message)
              assert False, error_message


  def unpauseSim(self):
      global cogval

      cogval = True
      rospy.logdebug("UNPAUSING START")
      rospy.wait_for_service('/gazebo/unpause_physics')
      rospy.logdebug("UNPAUSING service found...")
      unpaused_done = False
      counter = 0
      while not unpaused_done and not rospy.is_shutdown():
          if counter < self._max_retry:
              try:
                  rospy.logdebug("UNPAUSING service calling...")
                  self.unpause()
                  unpaused_done = True
                  rospy.logdebug("UNPAUSING service calling...DONE")
              except rospy.ServiceException as e:
                  counter += 1
                  rospy.logerr("/gazebo/unpause_physics service call failed...Retrying "+str(counter))
          else:
              error_message = "Maximum retries done"+str(self._max_retry)+", please check Gazebo unpause service"
              rospy.logerr(error_message)
              assert False, error_message

      rospy.logdebug("UNPAUSING FiNISH")


  def callback(self,data):
    global cogval
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

    masks = [mask1, mask2, mask3]

    mask = cv2.bitwise_or(mask1, mask2, mask1)

    cv_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_HSV2BGR)

    # plt.imshow(cv2.cvtColor(cv_image, cv2.COLOR_HSV2RGB))
    # plt.pause(0.001)

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

    # print("Error before is {}".format(error))
    error = (x_des - self.x_centroid)

    # if self.x_centroid!=0.0:
    #     error = (x_des - self.x_centroid)

    # else:
    #     return

    # print("Servoing error is {}, and depth value is {}".format(error, self.depval))

    vel_msg = Twist()
    while (abs(error)<150):
        received = rospy.get_param('unpause_sim')

        if received == False:
          self.pauseSim()
          print("Ready to receive information!")
          switch_pub.publish("Ready")
        
        if received == True:
          self.unpauseSim()
          print("Got info, unpausing simulation")

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
    return cogval


if __name__ == '__main__':

  cogval = True
  INTERACTION_TIME = 5
  rospy.init_node('cognition_module', anonymous=True)
  cog = cogmod()
  switch_pub = rospy.Publisher("/switcher", String, queue_size = 1)

  while not rospy.is_shutdown():
    continue
    # print("Cognitive now is {}".format(cogval))
    # if cogval:
    #   continue

    # else:
    #   switch_pub.publish("Ready")
      # print("Finished publishing switch")
        # print("Interacting with human for {}" .format((time.time() - init_time)))
      # print("Cog values is {}".format(cog.cognitive()))
      # cog.unpauseSim()
      # print("Cog values is {}".format(cog.cognitive()))
      # while (time.time() - init_time) < INTERACTION_TIME:
      #   print("Letting human pass for {}" .format((time.time() - init_time)))