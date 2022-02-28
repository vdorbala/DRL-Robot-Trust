from __future__ import division
import numpy as np
import rospy
import cv2

from std_msgs.msg import String

class confidence(object):
    def __init__(self, type):
    # save the subscriber object to a class member
        self.sub = rospy.Subscriber("/human_confidence", String, self.callback, queue_size=1)
        self.data = ""

    def callback(self,data):

        self.data = data.data

    def get_data(self):
            return self.data

    def unsubscribe(self):
    # use the saved subscriber object to unregister the subscriber
        self.sub.unregister()


class symbolsubclass(object):
    def __init__(self, class_type):
    # save the subscriber object to a class member
        if class_type == "human" or class_type == "Human":
            self.sub = rospy.Subscriber("/symbols", String, self.callback, queue_size=1)
        else:
            self.sub = rospy.Subscriber("/rob_symbols", String, self.callback, queue_size=1)
        self.symbols = None

    def callback(self,data):
        self.symbols = data.data

    def get_sym(self):
            return self.symbols

    def unsubscribe(self):
    # use the saved subscriber object to unregister the subscriber
        self.sub.unregister()



if __name__ == '__main__':

	# Initialize ros node
	rospy.init_node("state_machine", anonymous=True)

	# Initialize subscribers and publishers
	confobj = confidence()
	human_symb = symbolsubclass("human")
	robot_symb = symbolsubclass("robot")

	rospy.set_param('set_type', "robot")
    follow_type = rospy.get_param('set_type')

	sym_pub = rospy.Publisher("/selected_symbols", String, queue_size = 1)

	TRUST_LEVEL = 0.5
	
	conf_score = float(confobj.get_data())
	req_vel = velobj.get_data()

	while not rospy.is_shutdown():

		# Choosing Robot Symbols
		if  conf_score <= TRUST_LEVEL:
			req_symbols = robot_symb.get_data()
		# Choosing Human Symbols
		else:
			req_symbols = human_symb.get_data()

		sym_pub.publish(str(req_symbols))