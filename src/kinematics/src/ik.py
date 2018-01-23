#!/usr/bin/python

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
from ik_helper import *

ik_pub = rospy.Publisher('/ik',Float32MultiArray,queue_size = 1)

def ik_cb(msg):
	#p = np.matrix( [ msg.data[0]; msg.data[1]; msg.data[2] ] )
	x = msg.data[0]
	y = msg.data[1]
	z = msg.data[2]

	width_sps = Float32MultiArray(data = ik_legs(x,y,z) )
	ik_pub.publish(width_sps)


def ik():
	print "Inverse Kinematics Calculating..."
	rospy.init_node('ik')
	rospy.Subscriber('/joy_cmd', Float32MultiArray, ik_cb )
	rospy.spin()

if __name__ == '__main__':
	ik()


