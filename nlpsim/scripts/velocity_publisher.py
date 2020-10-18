import rospy
from geometry_msgs.msg import Twist

class velocity(object):
    def __init__(self, type):
    # save the subscriber object to a class member
        self.sub = rospy.Subscriber("/vel_cmds", Twist, self.callback, queue_size=1)
        self.data = ""

    def callback(self,data):

        self.data = data.data

    def get_data(self):
            return self.data

    def unsubscribe(self):
    # use the saved subscriber object to unregister the subscriber
        self.sub.unregister()


def vel_publish():
	
	vel_pub = rospy.Publisher("/robot_1/mobile_base/commands/velocity", Twist, queue_size = 1)
	

if __name__ == '__main__':
	rospy.init_node("send_vel")

	velobj = velocity()

	vel_publish()