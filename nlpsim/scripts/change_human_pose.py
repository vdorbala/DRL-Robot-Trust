#! /usr/bin/env python
from __future__ import division
import numpy as np
import rospy
import tf

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import *

from threading import Thread
from multiprocessing import Process
from multiprocessing.pool import ThreadPool as Pool
from geometry_msgs.msg import Pose

import argparse

class set_state(object):

  def __init__(self, i, res):
    print("In class!")
    self.name = i
    self.init_xpos = res.pose.position.x
    self.init_ypos = res.pose.position.y
    self.init_orient = res.pose.orientation.z
    print(res.pose.orientation.z)
    self.pose_publisher()

  def pose_publisher(self):
      pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
      pose_msg = ModelState()
      pose_msg.model_name = 'person{}'.format(self.name)
      rate = rospy.Rate(30)
      print(self.init_name, pose_msg.model_name, self.init_orient)

      while not rospy.is_shutdown():
          print(pose_msg.model_name)
          init_xpos = self.init_xpos
          init_ypos = self.init_ypos
          init_orient = self.init_orient
          for i in range(1,9):
            if self.name < 5:

              if abs(init_orient) > 3:
                while pose_msg.pose.position.y < 10:
                  pose_msg.pose.position.y += 4./100
                  pub.publish(pose_msg)
                  rate.sleep()
                pose_msg.pose.orientation.z = pose_msg.pose.orientation.z - np.pi

                while pose_msg.pose.position.y > -10:
                  pose_msg.pose.position.y -= 4./100
                  pub.publish(pose_msg)
                  rate.sleep()
                pose_msg.pose.orientation.z = pose_msg.pose.orientation.z - np.pi

              else:
                while pose_msg.pose.position.y > -10:
                  pose_msg.pose.position.y -= 4./100
                  pub.publish(pose_msg)
                  rate.sleep()
                pose_msg.pose.orientation.z = pose_msg.pose.orientation.z - np.pi

                while pose_msg.pose.position.y < 10:
                  pose_msg.pose.position.y += 4./100
                  pub.publish(pose_msg)
                  rate.sleep()
                pose_msg.pose.orientation.z = pose_msg.pose.orientation.z - np.pi

            else:

              if init_orient > 0:
                while pose_msg.pose.position.x < 10:
                  pose_msg.pose.position.x += 4./100
                  pub.publish(pose_msg)
                  rate.sleep()
                pose_msg.pose.orientation.z = pose_msg.pose.orientation.z - np.pi

                while pose_msg.pose.position.x > -10 :
                  pose_msg.pose.position.x -= 4./100
                  pub.publish(pose_msg)
                  rate.sleep()
                pose_msg.pose.orientation.z = pose_msg.pose.orientation.z - np.pi

              else:
                while pose_msg.pose.position.x > -10:
                  pose_msg.pose.position.x -= 4./100
                  pub.publish(pose_msg)
                  rate.sleep()
                pose_msg.pose.orientation.z = pose_msg.pose.orientation.z - np.pi

                while pose_msg.pose.position.x < 10:
                  pose_msg.pose.position.y += 4./100
                  pub.publish(pose_msg)
                  rate.sleep()
                pose_msg.pose.orientation.z = pose_msg.pose.orientation.z - np.pi


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

    init_orient = res.pose.orientation

    pose_msg.pose.position.x = res.pose.position.x
    pose_msg.pose.position.y = res.pose.position.y
    pose_msg.pose.orientation.z = res.pose.orientation.z
    (roll,pitch,theta) = tf.transformations.euler_from_quaternion([init_orient.x,init_orient.y,init_orient.z,init_orient.w])
    print(theta)

    while not rospy.is_shutdown():
        print(pose_msg.model_name)
        for i in range(1,9):
          if name < 5:

            if abs(theta) > 3:
              while pose_msg.pose.position.y < 10:
                pose_msg.pose.position.y += 4./100
                pub.publish(pose_msg)
                rate.sleep()

              pose_msg.pose.orientation.z = pose_msg.pose.orientation.z - np.pi/2

              while pose_msg.pose.position.y > -10:
                pose_msg.pose.position.y -= 4./100
                pub.publish(pose_msg)
                rate.sleep()

              pose_msg.pose.orientation.z = pose_msg.pose.orientation.z - np.pi/2

            else:
              while pose_msg.pose.position.y > -10:
                pose_msg.pose.position.y -= 4./100
                pub.publish(pose_msg)
                rate.sleep()
              pose_msg.pose.orientation.z = pose_msg.pose.orientation.z - np.pi/2

              while pose_msg.pose.position.y < 10:
                pose_msg.pose.position.y += 4./100
                pub.publish(pose_msg)
                rate.sleep()
              pose_msg.pose.orientation.z = pose_msg.pose.orientation.z - np.pi/2

          else:

            if theta > 0:
              while pose_msg.pose.position.x < 10:
                pose_msg.pose.position.x += 4./100
                pub.publish(pose_msg)
                rate.sleep()
              pose_msg.pose.orientation.z = pose_msg.pose.orientation.z - np.pi/2

              while pose_msg.pose.position.x > -10 :
                pose_msg.pose.position.x -= 4./100
                pub.publish(pose_msg)
                rate.sleep()
              pose_msg.pose.orientation.z = pose_msg.pose.orientation.z - np.pi/2

            else:
              while pose_msg.pose.position.x > -10:
                pose_msg.pose.position.x -= 4./100
                pub.publish(pose_msg)
                rate.sleep()
              pose_msg.pose.orientation.z = pose_msg.pose.orientation.z - np.pi/2

              while pose_msg.pose.position.x < 10:
                pose_msg.pose.position.y += 4./100
                pub.publish(pose_msg)
                rate.sleep()
              pose_msg.pose.orientation.z = pose_msg.pose.orientation.z - np.pi/2

def create_models():
  pose_msg = np.array([ModelState() for _ in range(10)])
  for i in range(1,9):
    pose_msg[i].model_name = 'person{}'.format(i)


if __name__ == '__main__':

      parser = argparse.ArgumentParser(description='Input the person number to move')

      parser.add_argument('person_num', metavar='N', type=int, nargs='+',
                          help='an integer for the accumulator')

      args = parser.parse_args()

      if not args:
        print("Please enter person number!")
        return 0

      rospy.init_node('pose_publisher', anonymous=True)

      try:
        res = gms_client("person{}".format(args.person_num), "link")
        pose_publisher_func(args.person_num, res)


      except rospy.ROSInterruptException:
          pass