#!/usr/bin/python
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
from ik_helper import *
from sensor_msgs.msg import Joy

pos_pub = rospy.Publisher('/des_pos',Float32MultiArray,queue_size = 1)
ort_pub = rospy.Publisher('/des_ort', Float32MultiArray, queue_size = 1)

max_radius = 7
class commandClass:
	def __init__(self,stage_):
		self.stage = stage_
		self.x = 0
		self.y = 0
		self.z = 92
		self.z_vec = 1
		self.command_xyz = [self.x,self.y,self.z]
		self.command_vec = [self.x, self.y, self.z_vec]
		self.nav = 0
	def updateXY(self,x,y,joy_rad):
		drad = axes_map(joy_rad,max_radius,0)
		r = np.sqrt(x**2+y**2)
		self.x = drad*(x/r)
		self.y = drad*(y/r)
	def updateZ(self,z):
		self.z = z
	def updateXYZ(self,x,y,slide):
		phi = ((slide+1)/2)*(np.radians(55))
		print np.degrees(phi)
		r = np.sqrt(x**2+y**2) 
		self.z_vec = np.cos(phi)
		r_corr = np.sin(phi)
		scale = r_corr/r
		self.x = scale*x
		self.y = scale*y
	def updateNav(self,nav):
		self.nav = nav
	def getNav(self):
		return self.nav
	def updateCommand_xyz(self):
		#self.command = ik_legs(self.x,self.y,self.z)
		self.command_xyz = [self.x,self.y,self.z]
	def updateCommand_vec(self):
		#self.command = ik_legs(self.x,self.y,self.z)
		self.command_vec = [self.x,self.y,self.z_vec]
	def getCommand_xyz(self):
		return self.command_xyz
	def getCommand_vec(self):
		return self.command_vec
	def getStage(self):
		return self.stage

command = [0]*10;
# z =92
# x= 0
# y= 0

z_min = 92
z_max = 173

def axes_map(js_sp,ax_max,ax_min):
	sp = (js_sp+1)/2.0
	out = (ax_max - ax_min)*sp+ax_min
	return out

def command_cb(msg):
	global command
	#Take joystick input to determine what navigation method to use
	if msg.buttons[3]:
		command1.updateNav(0)
		command2.updateNav(0)
		command3.updateNav(0)
	elif msg.buttons[4]:
		command1.updateNav(1)
		command2.updateNav(1)
		command3.updateNav(1)
	elif msg.buttons[5]:
		command1.updateNav(2)
		command2.updateNav(2)
		command3.updateNav(2)



	if command1.getNav() == 0:
		# XYZ
		if msg.buttons[8]:
			command3.updateZ(axes_map(msg.axes[3],z_max,z_min))
			command3.updateCommand_xyz()
		if msg.buttons[0]:
			command3.updateXY(-1*msg.axes[0],msg.axes[1],msg.axes[3])
			command3.updateCommand_xyz()

		if msg.buttons[9]:
			command2.updateZ(axes_map(msg.axes[3],z_max,z_min))
			command2.updateCommand_xyz()
		if msg.buttons[1]:
			command2.updateXY(-1*msg.axes[0],msg.axes[1],msg.axes[3])
			command2.updateCommand_xyz()

		if msg.buttons[11]:
			command1.updateZ(axes_map(msg.axes[3],z_max,z_min))
			command1.updateCommand_xyz()
		if msg.buttons[2]:
			command1.updateXY(-1*msg.axes[0],msg.axes[1],msg.axes[3])
			command1.updateCommand_xyz()

		command[0:3] = command1.getCommand_xyz()
		command[3:6] = command2.getCommand_xyz()
		command[6:9] = command3.getCommand_xyz()
		command[9] = command1.getNav()
		command_msg = Float32MultiArray(data = command)
		pos_pub.publish(command_msg)

	elif command1.getNav() == 1:
		# Vector motion

		# same as above, X and Y are set by axes 0 and 1, Z is set by axes 3
		# have to make sure X and Y are within a circle of radius 7
		if msg.buttons[11] or msg.buttons[9] or msg.buttons[8]:
			if msg.buttons[11]:
				command1.updateXYZ(-1*msg.axes[0],msg.axes[1],msg.axes[3])
				command1.updateCommand_vec()
				button = 11

			if msg.buttons[9]:
				command2.updateXYZ(-1*msg.axes[0],msg.axes[1],msg.axes[3])
				command2.updateCommand_vec()
				button = 9

			if msg.buttons[8]:
				command3.updateXYZ(-1*msg.axes[0],msg.axes[1],msg.axes[3])
				command3.updateCommand_vec()
				button = 8

			command[0:3] = command1.getCommand_vec()
			command[3:6] = command2.getCommand_vec()
			command[6:9] = command3.getCommand_vec()
			command[9] = button
			print command[6:9]
			command_msg = Float32MultiArray(data = command)
			ort_pub.publish(command_msg)

	#elif nav == 2: 
		# gradient descent




def ik():
	print "Listening to joystick..."
	rospy.init_node('joyinterpreter')
	rospy.Subscriber('/joy', Joy, command_cb)
	rospy.spin()

if __name__ == '__main__':
	command1 = commandClass(1)
	command2 = commandClass(2)
	command3 = commandClass(3)
	ik()
	


