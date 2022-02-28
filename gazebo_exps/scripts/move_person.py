#! /usr/bin/env python
from __future__ import division
import numpy as np
import rospy
import tf
import sys

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import *

from threading import Thread
from multiprocessing import Process
from multiprocessing.pool import ThreadPool as Pool
from geometry_msgs.msg import Pose

import thread

import argparse

def gms_client(model_name,relative_entity_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp1 = gms(model_name,relative_entity_name)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def pose_publisher_func(i, res):

    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=0)
    pose_msg = ModelState()
    pose_msg.model_name = 'person{}'.format(i)
    rate = rospy.Rate(30)
    name = i

    change_pose = Pose()
    init_orient = res.pose.orientation

    pose_msg.pose.position.x = res.pose.position.x
    pose_msg.pose.position.y = res.pose.position.y
    pose_msg.pose.orientation = res.pose.orientation
    (roll,pitch,theta) = tf.transformations.euler_from_quaternion([init_orient.x,init_orient.y,init_orient.z,init_orient.w])
    print(theta)

    while not rospy.is_shutdown():
        print(pose_msg.model_name)

        if int(name) < NUM_HORI:
          if abs(theta) > 3:
            while pose_msg.pose.position.y < ymax:
              pose_msg.pose.position.y += 4./100
              pub.publish(pose_msg)
              rate.sleep()

            theta = 0.0
            quaternion = tf.transformations.quaternion_from_euler(roll,pitch,theta)
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]

            while pose_msg.pose.position.y > -ymax:
              pose_msg.pose.position.y -= 4./100
              pub.publish(pose_msg)
              rate.sleep()

            theta = (np.pi)
            quaternion = tf.transformations.quaternion_from_euler(roll,pitch,theta)
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]

          else:
            while pose_msg.pose.position.y > -ymax:
              pose_msg.pose.position.y -= 4./100
              pub.publish(pose_msg)
              rate.sleep()

            theta = -(np.pi)
            quaternion = tf.transformations.quaternion_from_euler(roll,pitch,theta)
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]

            while pose_msg.pose.position.y < ymax:
              pose_msg.pose.position.y += 4./100
              pub.publish(pose_msg)
              rate.sleep()

            theta = 0.0
            quaternion = tf.transformations.quaternion_from_euler(roll,pitch,theta)
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]

        else:
          if theta > 0:
            while pose_msg.pose.position.x < xmax:
              pose_msg.pose.position.x += 4./100
              pub.publish(pose_msg)
              rate.sleep()

            theta = -(np.pi/2)
            quaternion = tf.transformations.quaternion_from_euler(roll,pitch,theta)
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]

            while pose_msg.pose.position.x > -xmax:
              pose_msg.pose.position.x -= 4./100
              pub.publish(pose_msg)
              rate.sleep()
            theta = (np.pi/2)
            quaternion = tf.transformations.quaternion_from_euler(roll,pitch,theta)
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]

          else:
            while pose_msg.pose.position.x > -xmax:
              pose_msg.pose.position.x -= 4./100
              pub.publish(pose_msg)
              rate.sleep()

            theta = (np.pi/2)
            quaternion = tf.transformations.quaternion_from_euler(roll,pitch,theta)
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]

            while pose_msg.pose.position.x < xmax:
              pose_msg.pose.position.x += 4./100
              pub.publish(pose_msg)
              rate.sleep()

            theta = -(np.pi/2)
            quaternion = tf.transformations.quaternion_from_euler(roll,pitch,theta)
            pose_msg.pose.orientation.x = quaternion[0]
            pose_msg.pose.orientation.y = quaternion[1]
            pose_msg.pose.orientation.z = quaternion[2]
            pose_msg.pose.orientation.w = quaternion[3]

if __name__ == '__main__':

      xmax = 22
      ymax = 17

      NUM_HORI = 10

      parser = argparse.ArgumentParser()
      parser.add_argument("--per")
      args = parser.parse_args()

      print(args.per)

      if args.per==None:
        print("Please enter person number!")
        sys.exit(0)

      per_num =  args.per

      rospy.init_node('pose_publisher', anonymous=True)

      try:
        res = gms_client("person{}".format(per_num), "link")
        # thread.start_new_thread( pose_publisher_func, (per_num, res, ) )
        pose_publisher_func(per_num, res)


      except rospy.ROSInterruptException:
          pass