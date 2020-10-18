#! /usr/bin/env python
import rospy

from nav_msgs.msg import Odometry
from std_msgs.msg import Header, String

import random

if __name__ == '__main__':

        rospy.init_node("goal_sender")

        rospy.set_param('goal_reached', True)

        sym_pub = rospy.Publisher('/rob_symbols', String, queue_size=0)

        x_size = 22
        y_size = 17

        symbollist = ['F', 'D', 'L', 'R']

        while not rospy.is_shutdown():

            reset = rospy.get_param("goal_reached")

            if reset:
                symblength = random.randint(4, 6)
                
                chosen = []

                for i in range(1, symblength):
                    chosen.append(random.choice(symbollist))

                chosen = str(chosen)

                rospy.set_param("goal_reached", False)

            sym_pub.publish(chosen)
