#!/usr/bin/env python
from __future__ import division
import numpy as np
import rospy
import cv2

import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import time

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import *

import send_goal
from std_srvs.srv import Empty

# from motion import forward, move

def gms_client(model_name,relative_entity_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp1 = gms(model_name,relative_entity_name)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


class detect_inter(object):
    def __init__(self):
    # save the subscriber object to a class member
        self.sub = rospy.Subscriber("/robot_1/base_scan", LaserScan, self.callback, queue_size=20)

        self.detected = False
        self.redetval = False

    def callback(self,data):
        global redet

        valuelist = np.array(data.ranges)
        min_angle = data.angle_min
        max_angle = data.angle_max

        increment = (max_angle - min_angle)

        longvalues = np.array(np.where(valuelist==np.inf))
        # print("Long values are {}".format(longvalues))

        left_idx = np.array(np.where(np.logical_and(longvalues<80, longvalues>30)))

        # print("Left count and count percentage are {}, {} resp.".format(len(left_idx[0]),(len(left_idx[0])/100)))

        right_idx = np.array(np.where(np.logical_and(longvalues<690, longvalues>530)))

        # print("Right count and count percentage are {}, {} resp.".format(len(right_idx[0]),(len(right_idx[0])/100)))

        det_perc = (len(right_idx[0])+ len(left_idx[0]))/100

        if det_perc>0.7:
            self.detected = True
        else:
            self.detected = False

        if det_perc == 0.0:
            self.redetval = True
            redet = True
        else:
            self.redetval = False
            redet = False

        # left = np.array(np.where(condition))
        # longvalues[longvalues>30 and longvalues<100]

    def found(self):

        return self.detected

    def redet(self):

        return self.redetval

    def unsubscribe(self):
    # use the saved subscriber object to unregister the subscriber
        self.sub.unregister()



class symbolsubclass(object):
    def __init__(self):
    # save the subscriber object to a class member
        self.sub = rospy.Subscriber("/symbols", String, self.callback, queue_size=20)
        self.symbols = None

    def callback(self,data):

        self.symbols = data.data

    def get_sym(self):
            return self.symbols

    def unsubscribe(self):
    # use the saved subscriber object to unregister the subscriber
        self.sub.unregister()


class kt_switch(object):
    def __init__(self):
    # save the subscriber object to a class member
        self.sub = rospy.Subscriber("/switcher", String, self.callback, queue_size=1)
        self.req = None

    def callback(self,data):
        print(data)
        self.req = data

    def get_req(self):
            return self.req

    def unsubscribe(self):
    # use the saved subscriber object to unregister the subscriber
        self.sub.unregister()

class robotposeclass(object):
    def __init__(self):
    # save the subscriber object to a class member
        self.sub = rospy.Subscriber("/robot_1/odom", Odometry, self.callback, queue_size=20)
        self.res = gms_client("robot_1", "link")
        self.pose = Odometry()

        x_gz = np.round(self.res.pose.position.x,3)
        y_gz = np.round(self.res.pose.position.y,3)
        w_gz = np.round(self.res.pose.orientation.w,3)

        self.pose_gz = [x_gz, y_gz, w_gz]

    def callback(self,data):


        x = np.round(data.pose.pose.position.x,3)
        y = np.round(data.pose.pose.position.y,3)
        w = np.round(data.pose.pose.orientation.w,3)

        self.pose = data
        # self.pose = [x,y,w]

    def getpose(self):
            # return self.pose_gz
            return self.pose

    def unsubscribe(self):
    # use the saved subscriber object to unregister the subscriber
        self.sub.unregister()


def gms_client(model_name,relative_entity_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp1 = gms(model_name,relative_entity_name)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def forward():
    velocity = Twist()
    velocity.linear.x = 0.3
    vel_pub.publish(velocity)

def forward_faster():
    velocity = Twist()
    velocity.linear.x = 0.5
    vel_pub.publish(velocity)


def move(pose_pub, vel_pub, cmd):

    res = gms_client("robot_1", "link")
    velocity = Twist()

    pose_msg = ModelState()
    pose_msg.model_name = 'robot_1'

    rate = rospy.Rate(50)

    robot = robotposeclass()

    theta_init = 0.0

    while theta_init==0.0:
        res = gms_client("robot_1", "link")

        init_orient = res.pose.orientation

        (roll,pitch,theta_init) = tf.transformations.euler_from_quaternion([init_orient.x,init_orient.y,init_orient.z,init_orient.w])

    sel_pos = [0.0, np.round(np.pi/2,3), np.round(-np.pi/2,3), np.round(np.pi,3)]

    if cmd == 'F' or cmd == 'U':
        print(theta_init)
        if abs(theta_init)<1:
            sel_val = 0
        elif np.isclose([-np.pi/2],[-np.pi/2+0.3], 0.3):
            sel_val = 2
        elif abs(theta_init)>3:
            sel_val = 3
        elif np.isclose([np.pi/2],[np.pi/2+0.3], 0.3):
            sel_val = 1

        time_init = time.time()
        if not firstcom:
            while (time.time() - time_init)<5:
                # print("Adjusting position!")
                forward()

            # print("Received forward command, going straight")
            # velocity = Twist()
            # time_init = time.time()
            # while (time.time() - time_init)<1:
            #     print("Stopping now!")
            #     vel_pub.publish(velocity)

        res = gms_client("robot_1", "link")
        init_orient = res.pose.orientation
        (roll,pitch,theta) = tf.transformations.euler_from_quaternion([init_orient.x,init_orient.y,init_orient.z,init_orient.w])

        theta_des = sel_pos[sel_val]
        quaternion = tf.transformations.quaternion_from_euler(roll,pitch,theta_des)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        pose_msg.pose.position.x = res.pose.position.x
        pose_msg.pose.position.y = res.pose.position.y

        time_init = time.time()
        while (time.time() - time_init)<5:
          print("Publishing Straight!")
          (roll, pitch, theta) = tf.transformations.euler_from_quaternion([pose_msg.pose.orientation.x,pose_msg.pose.orientation.y,pose_msg.pose.orientation.z,pose_msg.pose.orientation.w])
          print(pose_msg)
          pose_pub.publish(pose_msg)
          rate.sleep()


        # time_init = time.time()
        # while (time.time() - time_init)<5:
        #     # print("Adjusting position Right!")
        #     forward()

    elif cmd == 'L':
        if not firstcom:
            time_init = time.time()
            while (time.time() - time_init)<4:
                # print("Adjusting position Left!")
                forward()

        # Velocity Approach
        # velocity.linear.x = 0.0
        # velocity.angular.z = 0.2

        # time_init = time.time()
        # # while (time.time() - time_init)<2:
        # cur_orient = robot.getpose().pose.pose.orientation
        # (roll_cur, pitch_cur, theta_cur) = tf.transformations.euler_from_quaternion([cur_orient.x, cur_orient.y, cur_orient.z, cur_orient.w])
        # while (theta_cur-sel_pos[sel_val])<0.1:

        #     cur_orient = robot.getpose().pose.pose.orientation
        #     (roll_cur, pitch_cur, theta_cur) = tf.transformations.euler_from_quaternion([cur_orient.x, cur_orient.y, cur_orient.z, cur_orient.w])
        #     theta_cur = np.round(theta_cur,3)
        #     # print("Turning Left!")cur_orient.
        #     print(theta_cur, sel_pos[sel_val])
        #     vel_pub.publish(velocity)


        # Pose based approach
        res = gms_client("robot_1", "link")
        init_orient = res.pose.orientation
        (roll,pitch,theta) = tf.transformations.euler_from_quaternion([init_orient.x,init_orient.y,init_orient.z,init_orient.w])

        # sel_pos = [0.0, np.round(np.pi/2,3), np.round(-np.pi/2,3), np.round(np.pi,3)]

        if abs(theta)<1:
            sel_val = 2
        elif np.isclose(theta,[-np.pi/2+0.3], 0.3):
            sel_val = 3
        elif abs(theta_init)>3:
            sel_val = 1
        elif np.isclose(theta,[np.pi/2+0.3], 0.3):
            sel_val = 0

        theta_des = sel_pos[sel_val]
        quaternion = tf.transformations.quaternion_from_euler(roll,pitch,theta_des)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        pose_msg.pose.position.x = res.pose.position.x
        pose_msg.pose.position.y = res.pose.position.y

        time_init = time.time()
        while (time.time() - time_init)<5:
          print("Publishing Left!")
          (roll, pitch, theta) = tf.transformations.euler_from_quaternion([pose_msg.pose.orientation.x,pose_msg.pose.orientation.y,pose_msg.pose.orientation.z,pose_msg.pose.orientation.w])
          print(theta, theta_des)
          pose_pub.publish(pose_msg)
          rate.sleep()

        # velocity = Twist()
        # time_init = time.time()
        # while (time.time() - time_init)<1:
        #     print("Stopping now!")
        #     vel_pub.publish(velocity)

        time_init = time.time()
        while (time.time() - time_init)<4:
            # print("Adjusting position Right!")
            forward_faster()


    elif cmd == 'R':

        if not firstcom:
            time_init = time.time()
            while (time.time() - time_init)<4:
                # print("Adjusting position Right!")
                forward()

        # velocity.linear.x = 0.0
        # velocity.angular.z = 0.2

        # time_init = time.time()
        # # while (time.time() - time_init)<2:
        # cur_orient = robot.getpose().pose.pose.orientation
        # (roll_cur, pitch_cur, theta_cur) = tf.transformations.euler_from_quaternion([cur_orient.x, cur_orient.y, cur_orient.z, cur_orient.w])
        # theta_cur = np.round(theta_cur,3)
        # while (theta_cur-sel_pos[sel_val])<0.2:

        #     cur_orient = robot.getpose().pose.pose.orientation
        #     (roll_cur, pitch_cur, theta_cur) = tf.transformations.euler_from_quaternion([cur_orient.x, cur_orient.y, cur_orient.z, cur_orient.w])
        #     theta_cur = np.round(theta_cur,3)
        #     # print("Turning Left!")cur_orient.
        #     print(theta_cur, sel_pos[sel_val])
        #     vel_pub.publish(velocity)

        # time_init = time.time()
        # while (time.time() - time_init)<2:
        #     # print("Turning Right")
        #     vel_pub.publish(velocity)

        # Pose based approach
        res = gms_client("robot_1", "link")
        init_orient = res.pose.orientation
        (roll,pitch,theta) = tf.transformations.euler_from_quaternion([init_orient.x,init_orient.y,init_orient.z,init_orient.w])

        # sel_pos = [0.0, np.round(np.pi/2,3), np.round(-np.pi/2,3), np.round(np.pi,3)]

        if abs(theta)<1:
            sel_val = 2
        elif np.isclose(theta,[(-np.pi/2)+0.3], 0.3):
            sel_val = 3
        elif abs(theta_init)>3:
            sel_val = 1
        elif np.isclose(theta,[(np.pi/2)+0.3], 0.3):
            sel_val = 0

        theta_des = sel_pos[sel_val]
        quaternion = tf.transformations.quaternion_from_euler(roll,pitch,theta_des)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        pose_msg.pose.position.x = res.pose.position.x
        pose_msg.pose.position.y = res.pose.position.y

        time_init = time.time()
        while (time.time() - time_init)<5:
          print("Publishing Right! Angle is {}, {}".format(theta, theta_des))
          (roll, pitch, theta) = tf.transformations.euler_from_quaternion([pose_msg.pose.orientation.x,pose_msg.pose.orientation.y,pose_msg.pose.orientation.z,pose_msg.pose.orientation.w])
          pose_pub.publish(pose_msg)
          rate.sleep()



        # velocity = Twist()
        # time_init = time.time()
        # while (time.time() - time_init)<1:
        #     print("Stopping now!")
        #     vel_pub.publish(velocity)

        time_init = time.time()
        while (time.time() - time_init)<4:
            # print("Adjusting position Right!")
            forward_faster()


    elif cmd == 'D':

        if abs(theta_init)<1:
            sel_val = 3
        elif np.isclose([-np.pi/2],[-np.pi/2+0.3], 0.3):
            sel_val = 1
        elif abs(theta_init)>3:
            sel_val = 0
        elif np.isclose([np.pi/2],[np.pi/2+0.3], 0.3):
            sel_val = 2

        res = gms_client("robot_1", "link")
        init_orient = res.pose.orientation
        (roll,pitch,theta) = tf.transformations.euler_from_quaternion([init_orient.x,init_orient.y,init_orient.z,init_orient.w])

        theta_des = sel_pos[sel_val]
        quaternion = tf.transformations.quaternion_from_euler(roll,pitch,theta_des)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        pose_msg.pose.position.x = res.pose.position.x
        pose_msg.pose.position.y = res.pose.position.y

        time_init = time.time()
        while (time.time() - time_init)<5:
          print("Publishing message to change pose!")
          (roll, pitch, theta) = tf.transformations.euler_from_quaternion([pose_msg.pose.orientation.x,pose_msg.pose.orientation.y,pose_msg.pose.orientation.z,pose_msg.pose.orientation.w])
          print(pose_msg)
          pose_pub.publish(pose_msg)
          rate.sleep()



        # velocity = Twist()
        # time_init = time.time()
        # while (time.time() - time_init)<1:
        #     print("Stopping now!")
        #     vel_pub.publish(velocity)

        time_init = time.time()
        while (time.time() - time_init)<4:
            # print("Adjusting position Right!")
            forward()


    return False


if __name__ == '__main__':
    switch = kt_switch()

    # print("Waiting for switch")

    if "Ready" in str(switch.get_req()):
        print(" Knowledge transfer pipeline initiated")

    redet = True

    rospy.init_node("Intersection_navigator", anonymous=True)

    vel_pub = rospy.Publisher("/robot_1/mobile_base/commands/velocity", Twist, queue_size = 0)
    pose_pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=0)
    sym_subs = symbolsubclass()
    detect = detect_inter()

    while not sym_subs.get_sym():
        print("Waiting for symbols")

    CMD = str(sym_subs.get_sym())

    cmd_list = list(CMD)

    firstcom = True

    while not rospy.is_shutdown():
        print("REDET VAL IS {}".format(redet))
        if detect.found() and len(cmd_list)!=0 and not redet and not firstcom:
            print("At an intersection! Executing next command!")
            cmd = cmd_list.pop(0)
            print("Executing {}".format(cmd))
            move(pose_pub, vel_pub, cmd)
            # print("Sleeping, {}".format(redet))
            # time.sleep(5)

        elif len(cmd_list) == 0:
            print("No more commands to follow")
            break
        elif firstcom:
            cmd = cmd_list.pop(0)
            print("Executing first command! {}".format(cmd))
            move(pose_pub, vel_pub, cmd)
            firstcom = False

        else:
            forward()
        # else:
            # print("Still havent found what I'm looking for!")