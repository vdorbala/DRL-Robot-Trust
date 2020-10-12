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

        time_init = time.time()
        while (time.time() - time_init)<5:
            # print("Adjusting position Right!")
            forward()

    elif cmd == 'L':
        print(theta_init)
        if abs(theta_init)<1:
            sel_val = 1
        elif np.isclose([-np.pi/2],[-np.pi/2+0.3], 0.3):
            sel_val = 0
        elif abs(theta_init)>3:
            sel_val = 2
        elif np.isclose([np.pi/2],[np.pi/2+0.3], 0.3):
            sel_val = 3


        if not firstcom:
            time_init = time.time()
            while (time.time() - time_init)<4:
                # print("Adjusting position Left!")
                forward()

        # Velocity Approach

        print(sel_val)
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


    elif cmd == 'R':

        if abs(theta_init)<1:
            sel_val = 2
        elif np.isclose([-np.pi/2],[-np.pi/2+0.3], 0.3):
            sel_val = 3
        elif abs(theta_init)>3:
            sel_val = 1
        elif np.isclose([np.pi/2],[np.pi/2+0.3], 0.3):
            sel_val = 0

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
          print("Publishing Right! Angle is {}, {}".format(theta, theta_init))
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