#/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from math import pi

class OutAndBack():
	def __init__(self):
		rospy.init_node('out_and_back',anonymous=False)
		rospy.on_shutdown(self.shutdown)
		self.cmd_vel = rospy.Publisher('/cmd_vel',Twist)
	
	def shutdown(self):
		rospy.loginfo("stopping the robot...")

if __name__ == '__main__':
	try:
		OutAndBack()
	except:
		rospy.loginfo("out_and_back node terminated.")


