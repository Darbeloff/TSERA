#!/usr/bin/python

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
from ik_helper import *

ik_pub = rospy.Publisher('/ik',Float32MultiArray,queue_size = 1)


command = [0]*9;

def ik_cb(msg):
	#p = np.matrix( [ msg.data[0]; msg.data[1]; msg.data[2] ] )
	stage = int(msg.data[0])
	x = msg.data[1]
	y = msg.data[2]
	z = msg.data[3]

	width_sps = ik_legs(x,y,z)
	print x,y,z
	print width_sps
	
	
	stage_index1 = 3*(stage-1)
	stage_index2 = stage_index1+3

	command[ stage_index1 : stage_index2 ] = width_sps
	command_msg = Float32MultiArray(data = command)
	ik_pub.publish(command_msg)


def ik():
	print "Inverse Kinematics Calculating..."
	rospy.init_node('ik')
	#rospy.Subscriber('/joy_cmd', Float32MultiArray, ik_cb )
	rospy.Subscriber('/traj', Float32MultiArray, ik_cb )

	rospy.spin()

if __name__ == '__main__':
	ik()
	


