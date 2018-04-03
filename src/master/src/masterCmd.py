#!/usr/bin/python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String


master_pub = rospy.Publisher('/master', String ,queue_size = 1 )


def  masterJoy_cb(msg):
	if msg.buttons[0] == 1:
		master_pub.publish('first')
	if msg.buttons[1] == 1:
		master_pub.publish('second')


def masterCmd():
	rospy.init_node("master")
	rospy.Subscriber('/joy',Joy, masterJoy_cb)

	print 'Master Command Initialized...'
	rospy.spin()

if __name__ == '__main__':
	masterCmd()