#!/usr/bin/python


import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy

joy_pub = rospy.Publisher('/joy_cmd',Float32MultiArray,queue_size = 1)


def joy_cb(msg):
	sp_cmd = Float32MultiArray(data = [ msg.axes[0], msg.axes[1], msg.axes[2] ] )
	joy_pub.publish(sp_cmd)


def joy_cmd():
	print 'joy stick command initialized...'
	rospy.init_node('joyCmd')
	rospy.Subscriber('/joy', Joy, joy_cb )
	rospy.spin()


if __name__ == '__main__':
	joy_cmd()