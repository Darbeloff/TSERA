#!/usr/bin/python

import numpy as np

import roslib
import rospy

from std_msgs.msg import Float32MultiArray, String, Float32





class trajClass:
	def __init__(self):

		''' initialize the publisher and subscriber '''
		self.ik_pub = rospy.Publisher("/ik", Float32MultiArray, queue_size = 1)
		print 'Trajectory Generation Online'


		self.cmd_sub = rospy.Subscriber('/master',String, self.cmd_cb, queue_size = 1)
		self.error_sub = rospy.Subscriber('squareError',Float32, self.error_cb, queue_size = 1)

		self.next_setpoint = 1

	def cmd_cb(self,cmd):
		if cmd.data =='first':
			cmd.data = ''
			print 'Commanding Trajectory 1'
			i = 0
			inc = 1*np.pi/150
			amp = 5
			command = [0]*9
			while i<3*(2*np.pi)+inc:
				if self.next_setpoint ==1:
					
					command[6:9+1] =[5.0+amp*np.sin(i),5.0+amp*np.sin(i+2*np.pi/3.0),5.0+amp*np.sin(i-2*(np.pi)/3.0) ]
					command_msg = Float32MultiArray(data = command)
					self.ik_pub.publish(command_msg)


					i = i+inc


					rospy.sleep(0.05)
					self.next_setpoint=0

			print "Trajectory 1 Complete"
			command = [0]*9
			command_msg = Float32MultiArray(data = command)
			self.ik_pub.publish(command_msg)


		if cmd.data =='second':
			print 'Commanding Trajectory 2'
			i = 0
			inc = 2*np.pi/150
			amp = 5
			command = [0]*9
			while i<1*(2*np.pi)+inc:
				if self.next_setpoint ==1:
					
					command[3:6+1] =[5.0+amp*np.sin(i),5.0+amp*np.sin(i+2*np.pi/3.0),5.0+amp*np.sin(i-2*(np.pi)/3.0) ]
					command_msg = Float32MultiArray(data = command)
					self.ik_pub.publish(command_msg)


					i = i+inc


					rospy.sleep(0.05)
					self.next_setpoint=0

			print "Trajectory 2 Complete"
			command = [0]*9
			command_msg = Float32MultiArray(data = command)
			self.ik_pub.publish(command_msg)

	def error_cb(self, errorSq):
		if errorSq.data<=1:
			self.next_setpoint = 1
	



def trajCL():
	trajectory = trajClass()
	rospy.init_node("trajCL",anonymous = True)
	rospy.spin()

if __name__ == '__main__':
	trajCL()