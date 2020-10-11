#!/usr/bin/env python
from __future__ import division
import rospy
import cv2
import numpy as np
import re
from scipy.spatial import distance

from nav_msgs.msg import Odometry

from nlpsim.msg import Peoplepose

from yolov3_pytorch_ros.msg import BoundingBoxes, BoundingBox

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import *

import send_goal
from std_srvs.srv import Empty


def gms_client(model_name,relative_entity_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp1 = gms(model_name,relative_entity_name)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


class personposeclass(object):

    def __init__(self):
    # save the subscriber object to a class member
        self.sub = rospy.Subscriber("/person_pose",Peoplepose,self.callback, queue_size=20)

    def callback(self,data):
        name = str(data.name)
        x = np.round(data.xval,3)
        y = np.round(data.yval,3)
        yaw = np.round(data.yawval,3)
        number = re.findall("\d+", name)[0]

        if name not in namelist and [number,x,y,yaw] not in pose_list:
            namelist.append(name)
            pose_list.append([number, x, y, yaw])

    def unsubscribe(self):
    # use the saved subscriber object to unregister the subscriber
        self.sub.unregister()



class robotposeclass(object):
    def __init__(self):
    # save the subscriber object to a class member
        self.sub = rospy.Subscriber("/robot_1/odom", Odometry, self.callback, queue_size=20)
        self.res = gms_client("robot_1", "link")
        self.pose = []

        x_gz = np.round(self.res.pose.position.x,3)
        y_gz = np.round(self.res.pose.position.y,3)
        w_gz = np.round(self.res.pose.orientation.w,3)

        self.pose_gz = [x_gz, y_gz, w_gz]

    def callback(self,data):

        x = np.round(data.pose.pose.position.x,3)
        y = np.round(data.pose.pose.position.y,3)
        w = np.round(data.pose.pose.orientation.w,3)

        self.pose = [x,y,w]

    def getpose(self):
            return self.pose_gz
            # return self.pose

    def unsubscribe(self):
    # use the saved subscriber object to unregister the subscriber
        self.sub.unregister()



class objectdetectclass(object):
    def __init__(self):
    # save the subscriber object to a class member
        self.sub = rospy.Subscriber("/detected_objects_in_image", BoundingBoxes, self.callback, queue_size=20)
        self.pose = []
        self.objclass = ""
        self.probability = 0.0

        self.area = 0.0

    def callback(self,data):

        box = BoundingBoxes()

        if data.bounding_boxes:
            box = data.bounding_boxes[0]

        else:
            return 0

        self.objclass = box.Class

        self.probability = box.probability

        self.area = (box.xmax - box.xmin)*(box.ymax - box.ymin)

    def human_detect(self):

        if (self.objclass=="person"):
            return (self.area)

    def unsubscribe(self):
    # use the saved subscriber object to unregister the subscriber
        self.sub.unregister()




def main():

    global pose_list

    print("Retrieved poses successfully!")
    pose_list = np.array(pose_list)

    pose_list = pose_list[pose_list[:,0].argsort()]

    poses = pose_list[:, [1,2]]

    orientations = np.absolute(pose_list[:,[3]].astype(np.float))

    names = pose_list[:, 0]

    robotobj = robotposeclass()

    while not rospy.is_shutdown():

        while not robotobj.getpose():
            continue

        robot_pose = np.array(robotobj.getpose())

        x = robot_pose[0]
        y = robot_pose[1]
        w = np.round(robot_pose[2] - (np.pi/2), 3)

        detector = objectdetectclass()

        while not detector.human_detect():
            continue

        area = detector.human_detect()

        print("Area of first person detected is {}".format(area))
        print("Robot pose is {}, {}".format(robot_pose[0:2],w))

        contactval = distance.cdist([(x,y)], poses, 'euclidean')

        close_dist = np.where(contactval == np.min(contactval))[1]
        close_pose = pose_list[close_dist]
        close_orientation = orientations[close_dist]

        orientval = np.absolute(np.subtract(abs(w), close_orientation))

        print("Closest human is {}, who is {}m in front of the robot. \n Orientation is {}.".format(close_pose, np.min(contactval), np.min(orientval)))

        if np.min(contactval)<1.8 and orientval>np.pi/4 and area>30000:
            print("Sending stop command!")
            result = send_goal.movebase_client(0,0,0)
            pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
            if result:
                rospy.loginfo("Detected human and stopped!")

        print(orientval,np.pi/4)
        # orient_dist = np.where(orientval == np.min(orientval))[0]
        print("Orientation distances are {}".format(np.min(orientval)))

    return 0

if __name__ == '__main__':

    global sub
    
    NUM_PEOPLE = 7
    namelist = []
    pose_list = []
    rospy.init_node('interaction')

    # sub = rospy.Subscriber("/person_pose", Peoplepose, callback)
    # rospy.spin()
    personobj = personposeclass()

    while True:
        if len(namelist) >= NUM_PEOPLE:
            personobj.unsubscribe()
            break

    main()
