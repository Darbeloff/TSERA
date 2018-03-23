#!/usr/bin/python

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy


traj_pub = rospy.Publisher('/traj',Float32MultiArray,queue_size = 1)

#mostly garbage values!
hmin = 100
hmax = 177.85
inc = 1

def traj_cb(msg):

	if(msg.buttons[11]==1):# button 12
		print "Going home"
		sp_cmd = Float32MultiArray(data = [1, 0.001, 0.001, 60 ] ) #stage , x, y, z
		traj_pub.publish(sp_cmd)
		rospy.sleep(5)
		sp_cmd = Float32MultiArray(data = [2, 0.001, 0.001, 60 ] ) #stage , x, y, z
		traj_pub.publish(sp_cmd)
		rospy.sleep(5)
		sp_cmd = Float32MultiArray(data = [3, 0.001, 0.001, 60 ] ) #stage , x, y, z
		traj_pub.publish(sp_cmd)
		rospy.sleep(5)
		print "Arrived Home"

	elif (msg.buttons[10] == 1): # button 11 trigger
		print "Extension Trajectory"
		for j in range(1,3+1):
			for i in np.arange(hmin,hmax+inc,inc):
				sp_cmd = Float32MultiArray(data = [j, 0,0, i ] ) #stage , x, y, z
				traj_pub.publish(sp_cmd)
				rospy.sleep(0.07)
			rospy.sleep(1)
		
		rospy.sleep(2)

		for j in np.arange(3,0,-1):
			for i in np.arange(hmax,hmin-inc,-inc):
				sp_cmd = Float32MultiArray(data = [j, 0,0, i ] ) #stage , x, y, z
				traj_pub.publish(sp_cmd)
				rospy.sleep(0.07)
			rospy.sleep(1)

		print "Extension Trajectory Completed\n"
		rospy.sleep(10)

def traj_cmd():
	print 'Trajectory command initialized...'
	rospy.init_node('trajCmd')
	rospy.Subscriber('/joy', Joy, traj_cb )


if __name__ == '__main__':
	traj_cmd()
	rospy.spin()