#!/usr/bin/python
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
from ik_helper import *
from sensor_msgs.msg import Joy

ik_pub = rospy.Publisher('/ik',Float32MultiArray,queue_size = 1)


command = [0]*9;
z =91
x= 0
y= 0

z_min = 91
z_max = 174
def z_map(js_sp):
	sp = (js_sp+1)/2.0
	z = (z_max - z_min)*sp+z_min
	return z


def ik_cb(msg):
	global z
	

	max_radius = 4

	x = msg.axes[0]
	y = msg.axes[1]
	r = np.sqrt(x**2+y**2)



	if msg.buttons[0]:
		x = 5
		y = 5
	else:
		x= 0
		y= 0


	#constrain x,y setpoint to some max radius
	if r>=max_radius:
		x_corrected = max_radius*(x/r)
		y_corrected = max_radius*(y/r)
		x =x_corrected
		y = y_corrected


	width_sps = ik_legs(x,y,z)


	#logging
	if msg.buttons[7]:
		print width_sps, x,y,z

	if msg.buttons[6]:
		print 'x,y,z: ', x,y,z


	stage = 3
	stage_index1 = 3*(stage-1)
	stage_index2 = stage_index1+3


	if np.any(np.isnan(width_sps)):
		print "nan", x,y, z
	else:
		command[ stage_index1 : stage_index2 ] = width_sps
		command_msg = Float32MultiArray(data = command)
		ik_pub.publish(command_msg)


	#increment z
	z = z_map(msg.axes[3])


def ik():
	print "Inverse Kinematics Calculating..."
	rospy.init_node('ik')
	rospy.Subscriber('/joy', Joy, ik_cb )

	rospy.spin()

if __name__ == '__main__':
	ik()
	


