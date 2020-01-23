#!/usr/bin/env python

#Bates, Kyle
#2020-01-15

#borrowing from: 
#-https://andrewdai.co/xbox-controller-ros.html#rosjoy
#-http://wiki.ros.org/joy/Tutorials/WritingTeleopNode

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

X_LINEAR_MULT = 2;
Z_ANGULAR_MULT = 5;

def callback(data):
	twist = Twist()
	
	if (data.buttons[0] == 0):
		twist.linear.x = X_LINEAR_MULT * (data.axes[4] + data.axes[7])
		twist.angular.z = Z_ANGULAR_MULT * (data.axes[3] + data.axes[6])
	
	pub.publish(twist)
	
	

def joyPub():
		
	#output to vrep,
	global pub
	pub = rospy.Publisher("/vrep/twistCommand", Twist, queue_size=10)
	
	#input from joy,
	rospy.Subscriber("joy", Joy, callback)
	
	rospy.init_node("JoyToVREP")
	
	rospy.spin()
		
if __name__ == '__main__':
	joyPub()
