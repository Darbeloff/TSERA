#!/usr/bin/python

import numpy as np

import roslib
import rospy

from std_msgs.msg import Float32MultiArray, String





class trajClass:
	def __init__(self):

		''' initialize the publisher and subscriber '''
		self.ik_pub = rospy.Publisher("/ik", Float32MultiArray, queue_size = 1)
		print 'Trajectory Generation Online'


		self.cmd_sub = rospy.Subscriber('/master',String, self.cmd_cb, queue_size = 1)
		self.error_sub = rospy.Subscriber('squareError',Float32MultiArray, self.error_cb, queue_size = 1)

		self.next_setpoint = 1

	def cmd_cb(self,cmd):
		if cmd.data =='first':
			print 'Commanding Trajectory 3'
			i = 0
			command = [0]*9
			inc = 1.0/50.0
			g1 = 10.0

			while i<(g1+inc):
				if self.next_setpoint ==1:
					print command
					print '\n'
					command = [0,0,0, 0, 0, 0, i, i, i ]
					command_msg = Float32MultiArray(data = command)
					self.ik_pub.publish(command_msg)
					#increment i
					i = i + inc


					rospy.sleep(0.005)
					self.next_setpoint=0
			
			rospy.sleep(8)
			command = 9*[0];
			command_msg = Float32MultiArray(data = command)
			self.ik_pub.publish(command_msg)


			# print 'Commanding Trajectory 1'
			# i = 0
			# inc = 2.0*np.pi/250.0
			# amp = 4.5
			# command = [0]*9
			# while i<1/2.0*(2.0*np.pi)+inc:
			# 	if self.next_setpoint ==1:
					
			# 		command[6:9] =[5.0+amp*np.sin(i),5.0+amp*np.sin(i+2*np.pi/3.0),5.0+amp*np.sin(i-2*(np.pi)/3.0) ]
			# 		command_msg = Float32MultiArray(data = command)
			# 		self.ik_pub.publish(command_msg)
			# 		i = i+inc
			# 		self.next_setpoint=0

			# print "Trajectory 1 Complete"
			# command = [0]*9
			# command_msg = Float32MultiArray(data = command)
			# self.ik_pub.publish(command_msg)
			# cmd.data = ''




		if cmd.data =='second':
			print 'Commanding Trajectory 2'
			i = 0
			inc = 2.0*np.pi/250.0
			amp = 4.0
			command = [0]*9
			while i<(1*(2*np.pi)+inc):
				if self.next_setpoint ==1:
					command[3:6] =[5.0+amp*np.sin(i),5.0+amp*np.sin(i+2*np.pi/3.0),5.0+amp*np.sin(i-2*(np.pi)/3.0) ]
					command_msg = Float32MultiArray(data = command)
					self.ik_pub.publish(command_msg)


					i = i+inc


					self.next_setpoint=0

			print "Trajectory 2 Complete"
			command = [0]*9
			command_msg = Float32MultiArray(data = command)
			self.ik_pub.publish(command_msg)





		if cmd.data =='third':
			print 'Commanding Trajectory 3'
			i = 0
			command = [0]*9
			inc = 0.01
			g1 = 5.0
			g2 = 5.0

			while i<(g1+inc):
				if self.next_setpoint ==1:
					print command
					print '\n'
					command = [i,i,i, 0, 0, 0, 0, 0, 0 ]
					command_msg = Float32MultiArray(data = command)
					self.ik_pub.publish(command_msg)
					#increment i
					i = i + inc


					rospy.sleep(0.01)
					self.next_setpoint=0
			i = 0
			while i<(g2+inc):
				if self.next_setpoint ==1:
					print command
					print '\n'
					command = [g1,g1,g1, i, i, i, 0, 0, 0 ]
					command_msg = Float32MultiArray(data = command)
					self.ik_pub.publish(command_msg)
					#increment i
					i = i + inc
			i = 0
			while i<(g2+inc):
				if self.next_setpoint ==1:
					print command
					print '\n'
					command = [g1, g1, g1, g2, g2, g2, i, i, i ]
					command_msg = Float32MultiArray(data = command)
					self.ik_pub.publish(command_msg)
					#increment i
					i = i + inc


					rospy.sleep(0.01)
					self.next_setpoint=0

			print "Trajectory 3 Complete"

			rospy.sleep(10)
			command = [0]*9
			command_msg = Float32MultiArray(data = command)
			self.ik_pub.publish(command_msg)


	def error_cb(self, errorSq):
		if errorSq.data<=0.5:
			self.next_setpoint = 1
	



def trajCL():
	trajectory = trajClass()
	rospy.init_node("trajCL",anonymous = True)
	rospy.spin()

if __name__ == '__main__':
	trajCL()