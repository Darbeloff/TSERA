#!/usr/bin/python

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy


js_traj_pub = rospy.Publisher('/ik',Float32MultiArray,queue_size = 1)



js_cmd = [0]*9

inc = 2*np.pi/150
amp = 4.5

def js_traj_cb(msg):


	if (msg.buttons[0] == 1): # trigger start
		amp = 5

		print "Sine Trajectory Start"

		#stage 3
		for i in np.arange(0,3*(2*np.pi)+inc,inc):
			stage_command = [5.0+amp*np.sin(i),5.0+amp*np.sin(i+2*np.pi/3.0),5.0+amp*np.sin(i-2*(np.pi)/3.0) ]
			js_cmd[6:9] = stage_command
			js_cmd_msg = Float32MultiArray(data = js_cmd ) #stage , x, y, z
			js_traj_pub.publish(js_cmd_msg)
			rospy.sleep(0.09)
		rospy.sleep(5)
		
		stage_command = [0,0,0]
		js_cmd[6:9] = stage_command
		js_cmd_msg = Float32MultiArray(data = js_cmd ) #stage , x, y, z
		js_traj_pub.publish(js_cmd_msg)

		rospy.sleep(15)


		#stage 2
		amp = 5
		for i in np.arange(0,3*(2*np.pi)+inc,inc):
			stage_command = [5.0+amp*np.sin(i),5.0+amp*np.sin(i+2*np.pi/3.0),5.0+amp*np.sin(i-2*(np.pi)/3.0) ]
			js_cmd[3:6] = stage_command
			js_cmd_msg = Float32MultiArray(data = js_cmd ) #stage , x, y, z
			js_traj_pub.publish(js_cmd_msg)
			rospy.sleep(0.09)

		rospy.sleep(5)

		stage_command = [0,0,0]
		js_cmd[3:6] = stage_command
		js_cmd_msg = Float32MultiArray(data = js_cmd ) #stage , x, y, z
		js_traj_pub.publish(js_cmd_msg)

		rospy.sleep(15)

		# #stage 1
		# amp = 4.5
		# for i in np.arange(0,3*(2*np.pi)+inc,inc):
		# 	stage_command = [5.0+amp*np.sin(i),5.0+amp*np.sin(i+2*np.pi/3.0),5.0+amp*np.sin(i-2*(np.pi)/3.0) ]
		# 	js_cmd[0:3] = stage_command
		# 	js_cmd_msg = Float32MultiArray(data = js_cmd ) #stage , x, y, z
		# 	js_traj_pub.publish(js_cmd_msg)
		# 	rospy.sleep(0.09)

		# rospy.sleep(5)

		stage_command = [0,0,0]
		js_cmd[0:3] = stage_command
		js_cmd_msg = Float32MultiArray(data = js_cmd ) #stage , x, y, z
		js_traj_pub.publish(js_cmd_msg)
		

		print "Sin Trajectory Completed\n"
		rospy.sleep(100)


	elif msg.buttons[4]==1:
		print "Extend Trajectory Start"

		#all stages
		for j in np.arange(0,3):
			stageStart  = 3*j
			stageEnd = stageStart + 3
			for i in np.arange(0,3*(2*np.pi)+inc,inc):
				stage_command = [10.5,10.5,10.5]
				js_cmd[stageStart:stageEnd] = stage_command
				js_cmd_msg = Float32MultiArray(data = js_cmd ) #stage , x, y, z
				js_traj_pub.publish(js_cmd_msg)
				rospy.sleep(0.09)
			rospy.sleep(7)

		print "Extend Trajectory End"


def js_traj_cmd():
	print 'Joint Space Trajectory command initialized...'
	rospy.init_node('JStrajCmd')
	rospy.Subscriber('/joy', Joy, js_traj_cb )


if __name__ == '__main__':
	js_traj_cmd()
	rospy.spin()