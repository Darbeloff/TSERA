#!/usr/bin/python
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
from sympy import *
#from ik_helper import *
#from sensor_msgs.msg import Joy

ort_pub = rospy.Publisher("/des_ort_xyz", Float32MultiArray, queue_size = 1)

x, y, Lt = symbols('x y Lt')
b_x = (sqrt((-6*y**2)+2*(sqrt(3)*Lt+3*x)*(-x+sqrt(x**2+y**2))))/(sqrt(Lt**2))
b_y = ((x+sqrt(x**2+y**2))*sqrt(-6*y**2+2(sqrt(3)*Lt+3*x)*(-x+sqrt(x**2+y**2))))/(sqrt(Lt**2)*y)
b_z = (Lt-2*sqrt(3)*sqrt(x**2+y**2))/(sqrt(Lt**2))


class positionClass():
	def __init__(self, stage_):
		self.x = 0
		self.y = 0
		self.z = 92
		self.count = 0
		self.list = [0]*100
		self.grad_cont = True
	def updateXYZ(self,x,y,z):
		self.x = x
		self.y = y
		self.z = z
	def x(self):
		return self.x
	def y(self):
		return self.y
	def z(self):
		return self.z
	def updatelist(self, xyz):
		if self.list[0] == 0:
			self.list[0] = xyz
		elif self.list[-1] == 0:
			self.list[self.count] = xyz
	def updatepos(self):
		self.count += 1
	def position(self):
		return self.count
	def J(self, T, y):
		if abs(y) > 0.1:
			b_vector = [b_x, b_y, b_z]
		else
			b_y = -b_x*b_z
			b_vector = [b_x, b_y, b_z]
		J = np.dot(b_vector, T)
		return J
	def arrived(self):
		self.list.pop(0)
		xyz_msg = Float32MultiArray(data = self.list[0])
		ort_pub.publish(xyz_msg)
	def cont(self):
		return self.grad_cont

position = positionClass()
alpha = 0.5

def gradient_ascent(unit_vector):
	
	#while position.position() < 10
	position.grad_cont = True
	x, y, Lt = symbols('x y Lt')
	J = position.J(unit_vector, position.y())
	djdx = diff(J, x)
	djdy = diff(J, y)
	while position.cont() == True:
		new_x = position.x() + djdx.subs([(x, position.x()),(y,position.y()), (Lt, 85.3)])*alpha
		new_y = position.y() + djdy.subs([(x,position.x()),(y,position.y()), (Lt, 85.3)])*alpha
		new_z = position.z() #This can't be correct
		position.updateXYZ(new_x,new_y,new_z)
		xyz = [position.x(),position.y(),position.z()]
		position.updatelist(xyz)
		position.updatepos()
		xyz_msg = Float32MultiArray(data = xyz)
		ort_pub.publish(xyz_msg)

def ort_callback(msg):
	xyz = [0]*3
	magnitude = np.sqrt(((msg.data[0])**2) + ((msg.data[1])**2) + ((msg.data[2])**2))
	unit_vector = [msg.data[0], msg.data[1], msg.data[2]]/magnitude
	position.grad_cont = False
	if position.position() != 100:
		gradient_ascent(unit_vector)

def waypoint_callback(msg):
	position.arrived()

def orientation():
	rospy.init_node('grad_calc')
	rospy.Subscriber("/des_ort", Float32MultiArray, ort_callback)
	rospy.Subscriber("/continueWaypoint", Bool, waypoint_callback)
	rospy.spin()

if __name__ = '__main__':
	orientation()
