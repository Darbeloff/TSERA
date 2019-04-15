#!/usr/bin/python
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
from ik_helper import *
from sensor_msgs.msg import Joy

ik_pub = rospy.Publisher('/ik',Float32MultiArray,queue_size = 1)

max_radius = 7
class commandClass:
	def __init__(self,stage_):
		self.stage = stage_
		self.x = 0
		self.y = 0
		self.z = 120
		self.rotated = 0
		self.command = ik_legs(self.x,self.y,self.z, self.rotated)
	def updateXYZ(self,x,y,z,rotated):
		self.x = x
		self.y = y
		self.z = z
		self.rotated = rotated
	def updateCommand(self):
		self.command = ik_legs(self.x,self.y,self.z, self.rotated)
	def getCommand(self):
		return self.command
	def getStage(self):
		return self.stage

command = [0]*9;

z_min = 92
z_max = 173
def axes_map(js_sp,ax_max,ax_min):
	sp = (js_sp+1)/2.0
	out = (ax_max - ax_min)*sp+ax_min
	return out

def ik_cb(msg):
	global command

	command1.updateXYZ(msg.data[0], msg.data[1], msg.data[2])
	command1.updateCommand()
	command2.updateXYZ(msg.data[3], msg.data[4], msg.data[5])
	command2.updateCommand()
	command3.updateXYZ(msg.data[6], msg.data[7], msg.data[8])
	command3.updateCommand()

	command[0:3] = command1.getCommand()
	command[3:6] = command2.getCommand()
	command[6:] = command3.getCommand()



	command_msg = Float32MultiArray(data = command)
	ik_pub.publish(command_msg)

def ik_ort_cb(msg):
	global command
	#Need to add way to not accept incorrect inputs
	command1.updateXYZ(msg.data[0], msg.data[1], msg.data[2], msg.data[3])
	command1.updateCommand()
	command2.updateXYZ(msg.data[4], msg.data[5], msg.data[6], msg.data[7])
	command2.updateCommand()
	command3.updateXYZ(msg.data[8], msg.data[9], msg.data[10], msg.data[11])
	command3.updateCommand()

	command[0:3] = command1.getCommand()
	command[3:6] = command2.getCommand()
	command[6:] = command3.getCommand()

	command_msg = Float32MultiArray(data = command)
	ik_pub.publish(command_msg)	

def ik():
	print "Inverse Kinematics Calculating..."
	rospy.init_node('ik')
	rospy.Subscriber('/des_pos', Float32MultiArray, ik_cb )
	rospy.Subscriber('/des_ort_xyz', Float32MultiArray, ik_ort_cb )
	rospy.spin()

if __name__ == '__main__':
	command1 = commandClass(1)
	command2 = commandClass(2)
	command3 = commandClass(3)
	ik()
	


