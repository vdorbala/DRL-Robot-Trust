#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import re
from scipy.spatial import distance

from nav_msgs.msg import Odometry

from nlpsim.msg import Peoplepose

from yolov3_pytorch_ros.msg import BoundingBoxes, BoundingBox


class personposeclass(object):

    def __init__(self):
    # save the subscriber object to a class member
        self.sub = rospy.Subscriber("/person_pose",Peoplepose,self.callback, queue_size=20)

    def callback(self,data):
        name = str(data.name)
        x = np.round(data.xval,3)
        y = np.round(data.yval,3)
        yaw = np.round(data.yawval,3)

        if name not in namelist and [x,y] not in pose_list:
            number = re.findall("\d+", name)[0]
            namelist.append(name)
            pose_list.append([number,x,y,yaw])

    def unsubscribe(self):
    # use the saved subscriber object to unregister the subscriber
        self.sub.unregister()




class robotposeclass(object):
    def __init__(self):
    # save the subscriber object to a class member
        self.sub = rospy.Subscriber("/robot_1/odom",Odometry, self.callback, queue_size=20)
        self.pose = []

    def callback(self,data):

        x = np.round(data.pose.pose.position.x,3)
        y = np.round(data.pose.pose.position.y,3)
        w = np.round(data.pose.pose.orientation.w,3)

        self.pose = [x,y,w]

    def getpose(self):
            return self.pose

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

    def callback(self,data):

        box = BoundingBoxes()

        box = data.bounding_boxes[0]

        self.objclass = box.Class

        self.probability = box.probability

    def human_detect(self):

        if (self.objclass=="human"):
            return (self.probability)

    def unsubscribe(self):
    # use the saved subscriber object to unregister the subscriber
        self.sub.unregister()


def main():

    global pose_list

    print("Reached main successfully!")

    pose_list = np.array(sorted(pose_list, key=lambda x: (x[0],x[1])))

    poses = pose_list[:, [1,2]]

    names = pose_list[:,0]

    robotobj = robotposeclass()

    while not robotobj.getpose():
        continue

    pose = np.array(robotobj.getpose())

    x = pose[0]
    y = pose[1]
    w = pose[2]

    detector = objectdetectclass()

    while not detector.human_detect():
        continue

    print(detector.human_detect())

    print("Robot pose is {}".format(pose[0:2]))

    contactval = distance.cdist([(x,y)], poses, 'euclidean')

    close_human = names[np.where(contactval == np.min(contactval))[1]]

    print("Closest human is {}".format(close_human))

    return 0


if __name__ == '__main__':

    NUM_PEOPLE = 7
    
    global sub

    namelist = []
    
    pose_list = []

    rospy.init_node('interaction')


    # sub = rospy.Subscriber("/person_pose", Peoplepose, callback)
    # rospy.spin()

    personobj = personposeclass()

    while True:
        if len(namelist) >= NUM_PEOPLE:
            print("UNREGISTERING Subscriber!")
            personobj.unsubscribe()
            break

    main()

    # if sub != 0:
    #     main(
