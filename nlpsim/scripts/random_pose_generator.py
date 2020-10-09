#!/usr/bin/env python
import subprocess
import random
import os
import rospy
import numpy as np

from gazebo_msgs.msg import ModelState
from nlpsim.msg import Peoplepose

def spawn():

	xvalsh = []
	yvalsh = []

	xvalsv = []
	yvalsv = []

	yawvalh = []
	yawvalv = []

	row_split = (hum_hor*width)

	column_split = (hum_ver*length)

	x_size = 22
	y_size = 17
	count  = 1
     
	for i in range(1, row_split):
		num = int(row_split/hum_hor)
		xvalsh.append(random.uniform(-x_size, x_size))
		yvalsh.append(-10.5 + 6*num + random.uniform(-0.1, 0.05))

	for i in range(1, column_split):
		num = int(column_split/hum_ver)
		xvalsv.append(-18 + 6*num + random.uniform(-0.1, 0.05))
		yvalsv.append(random.uniform(-y_size, y_size))

	# for i in range(1,3):
	# 	xvalsh.append(random.uniform(-x_size, x_size))
	# 	yvalsh.append(2.0 + random.uniform(-0.1, 0.05))

	# for i in range(3,5):
	# 	xvalsh.append(random.uniform(-9, 9))
	# 	yvalsh.append(-5.0 + random.uniform(-0.1, 0.05))

	for i in range(1,row_split + 1):
		yawvalh.append(random.choice([np.pi, 0]))

	# for i in range(5,7):
	# 	xvalsv.append(0.76 + random.uniform(-0.1, 0.05))
	# 	yvalsv.append(random.uniform(-9, 9))

	# for i in range(7,9):
	# 	xvalsv.append(-5.0 + random.uniform(-0.1, 0.05))
	# 	yvalsv.append(random.uniform(-9, 9))


	for i in range(1, column_split + 1):
		yawvalv.append(random.choice([-np.pi/2, np.pi/2]))

	# Horizontal
	for i in range(1, row_split):

		# spawn = subprocess.Popen("rosrun gazebo_ros spawn_model -database person_standing -sdf -model person{} -y {} -x {} -Y {}".format(i, xvalsh[i-1], yvalsh[i-1], yawval[i-1] ),stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)

		spawn = subprocess.Popen("rosrun gazebo_ros spawn_model -database person_walking_nocollide -sdf -model person{} -y {} -x {} -Y {}".format(i, xvalsh[i-1], yvalsh[i-1], yawvalh[i-1]),stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)

	# Vertical
	for i in range(5, column_split):

		spawn = subprocess.Popen("rosrun gazebo_ros spawn_model -database person_walking_nocollide -sdf -model person{} -y {} -x {} -Y {}".format(i,  xvalsv[i-1], yvalsv[i-1], yawvalv[i-1]),stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)


	while not rospy.is_shutdown():

		for i in range(1, 5):
			msg = Peoplepose()
			msg.name = "person{}".format(i)
			msg.xval = float(xvalsh[i-1])
			msg.yval = float(yvalsh[i-1])

			msg.yawval = float(yawvalh[i-1])
			# print(type(msg.xval))

			person_pose.publish(msg)

		for i in range(5, 9):
			msg = Peoplepose()
			msg.name = "person{}".format(i)
			msg.xval = float(xvalsv[i-1])
			msg.yval = float(yvalsv[i-1])

			msg.yawval = float(yawvalv[i-1])

			person_pose.publish(msg)


	return 0

if __name__ == '__main__':
	
	rospy.init_node("random_spawn")

	hum_hor = 2

	hum_ver = 2

	width = 6

	length = 8

	person_pose = rospy.Publisher('/person_pose', Peoplepose, queue_size=10)
	
	spawn()