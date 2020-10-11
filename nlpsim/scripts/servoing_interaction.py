#!/usr/bin/env python
from __future__ import division
import rospy
import cv2
import numpy as np
import re
from scipy.spatial import distance

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from nlpsim.msg import Peoplepose

from yolov3_pytorch_ros.msg import BoundingBoxes, BoundingBox

import send_goal
from std_srvs.srv import Empty



class objectdetectclass(object):
    def __init__(self):
    # save the subscriber object to a class member
        self.sub = rospy.Subscriber("/detected_objects_in_image", BoundingBoxes, self.callback, queue_size=20)
        self.pose = []
        self.objclass = ""
        self.probability = 0.0

        self.area = 0.0
        self.box = BoundingBoxes()

    def callback(self,data):
        if data.bounding_boxes:
            self.box = data.bounding_boxes[0]

        else:
            return 0

        self.objclass = self.box.Class

        self.probability = self.box.probability

        self.area = (self.box.xmax - self.box.xmin)*(self.box.ymax - self.box.ymin)

    def human_detect(self):
        if (self.objclass=="person"):
            return (self.box)

    def unsubscribe(self):
    # use the saved subscriber object to unregister the subscriber
        self.sub.unregister()




class people_detect(object):
    
    def __init__(self):
        self.depth_sub = rospy.Subscriber("/robot_1/camera/depth/image_raw", Image, self.depth_callback, queue_size=20)
        self.image_sub = rospy.Subscriber("/robot_1/camera/rgb/image_raw", Image, self.rgb_callback, queue_size=20)

    def depth_callback(self, data):
        return data

    def rgb_callback(self, data):
        return data
        
    def unsubscribe(self):
    # use the saved subscriber object to unregister the subscriber
        self.depth_sub.unregister()
        self.image_sub.unregister()


class cube_detect():

    def __init__(self):
        self.depth_sub = rospy.Subscriber("/robot_1/camera/depth/image_raw", Image, self.depth_callback, queue_size=20)
        self.image_sub = rospy.Subscriber("/robot_1/camera/rgb/image_raw", Image, self.rgb_callback, queue_size=20)

    def depth_callback(self, data):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

        return data

    def rgb_callback(self, data):

        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret,thresh = cv2.threshold(gray,127,255,1)

        contours,h = cv2.findContours(thresh,1,2)

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
            print len(approx)
            if len(approx)==5:
                print "pentagon"
                cv2.drawContours(img,[cnt],0,255,-1)
            elif len(approx)==3:
                print "triangle"
                cv2.drawContours(img,[cnt],0,(0,255,0),-1)
            elif len(approx)==4:
                print "square"
                cv2.drawContours(img,[cnt],0,(0,0,255),-1)
            elif len(approx) == 9:
                print "half-circle"
                cv2.drawContours(img,[cnt],0,(255,255,0),-1)
            elif len(approx) > 15:
                print "circle"
                cv2.drawContours(img,[cnt],0,(0,255,255),-1)

        cv2.imshow('img',img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        print(cnt)
        
        return data
        
    def unsubscribe(self):
    # use the saved subscriber object to unregister the subscriber
        self.depth_sub.unregister()
        self.image_sub.unregister()

def track_and_move():

    while not rospy.is_shutdown():

        box = BoundingBoxes()

        detect = cube_detect()
        # human = people_detect()
        # detect = objectdetectclass()

        while not detect.rgb_callback():
            continue

        # box = detect.rgb_callback()

        # xmax = box.xmax
        # ymax = box.ymax
        # xmin = box.xmin
        # ymin = box.ymin

        # center_point = [(xmax - xmin)/2, (ymax - ymin)/2]

        # x_coord = center_point[0]

        # print("Sending stop command!")
        # result = send_goal.movebase_client(0,0,0)
        # pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        # if result:
        #     rospy.loginfo("Detected human and stopped!")

        # if x_coord<(image_hor_dim/2):
        #     pass
        # else:
        #     pass


if __name__ == '__main__':

    rospy.init_node('servoing_interaction')

    track_and_move()
