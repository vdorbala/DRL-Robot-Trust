import numpy as np
import rospy
from geometry_msgs.msg import Twist
import time

def main():

	while True:
		phrase = str(raw_input("Enter what you would like the robot to do. \n"))

		if 'forward' in phrase:
			cmdmsg = Twist()
			cmdmsg.linear.x = 5
			act.publish(cmdmsg)
			time.sleep(0.05)
		else:
			print("Nothing works!")
		# rospy.spin()

if __name__ == '__main__':
	rospy.init_node('velocitty_publisher', anonymous=True)
	act = rospy.Publisher("/cmd_vel",Twist, queue_size=0)
	main()