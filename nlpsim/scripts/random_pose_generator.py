#!/usr/bin/env python
import subprocess
import random
import os
import rospy
import np

from gazebo_msgs.msg import ModelState
from nlpsim.msg import Peoplepose

def spawn():

	xvalsh = []
	yvalsh = []

	xvalsv = []
	yvalsv = []

	yawval = []

	for i in range(1,3):
		xvalsh.append(random.uniform(-9, 9))
		yvalsh.append(2.0 + random.uniform(-0.1, 0.05))

	for i in range(3,5):
		xvalsh.append(random.uniform(-9, 9))
		yvalsh.append(-5.0 + random.uniform(-0.1, 0.05))

	for i in range(5,7):
		xvalsv.append(0.76 + random.uniform(-0.1, 0.05))
		yvalsv.append(random.uniform(-9, 9))

	for i in range(7,9):
		xvalsv.append(-5.0 + random.uniform(-0.1, 0.05))
		yvalsv.append(random.uniform(-9, 9))

	for i in range(1,9):
		yawval.append(random.uniform(-np.pi,np.pi))

	for i in range(1, 5):

		spawn = subprocess.Popen("rosrun gazebo_ros spawn_model -database person_standing -sdf -model person{} -y {} -x {} -yaw {}".format(i, xvalsh[i-1], yvalsh[i-1], yawval[i-1] ),stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)


	for i in range(5, 9):

		spawn = subprocess.Popen("rosrun gazebo_ros spawn_model -database person_standing -sdf -model person{} -y {} -x {} -yaw {}".format(i,  xvalsv[i-5], yvalsv[i-5], yawval[i]),stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)


	while not rospy.is_shutdown():

		for i in range(1, 5):
			msg = Peoplepose()
			msg.name = "person{}".format(i)
			msg.xval = float(xvalsh[i-1])
			msg.yval = float(yvalsh[i-1])

			msg.yawval = float(yawval[i-1])
			# print(type(msg.xval))

			person_pose.publish(msg)

		for i in range(5, 9):
			msg = Peoplepose()
			msg.name = "person{}".format(i)
			msg.xval = float(xvalsv[i-5])
			msg.yval = float(yvalsv[i-5])

			msg.yawval = float(yawval[i])

			person_pose.publish(msg)


	return 0

if __name__ == '__main__':
	
	rospy.init_node("random_spawn")

	person_pose = rospy.Publisher('/person_pose', Peoplepose, queue_size=10)
	
	spawn()