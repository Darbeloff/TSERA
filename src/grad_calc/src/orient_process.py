#!/usr/bin/python
import rospy
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np
from sympy import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
#from ik_helper import *
#from sensor_msgs.msg import Joy

ort_pub = rospy.Publisher("/des_ort_xyz", Float32MultiArray, queue_size = 1)

x, y, Lt = symbols('x y Lt')
b_x = (sqrt((-6*y**2)+2*(sqrt(3)*Lt+3*x)*(-x+sqrt(x**2+y**2))))/(sqrt(Lt**2))
b_y = ((x+sqrt(x**2+y**2))*sqrt(-6*y**2+2*(sqrt(3)*Lt+3*x)*(-x+sqrt(x**2+y**2))))/(sqrt(Lt**2)*y)
b_z = (Lt-2*sqrt(3)*sqrt(x**2+y**2))/(sqrt(Lt**2))


class positionClass():
	def __init__(self):
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
		else:
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
	def clearlist(self):
		self.list = [0]*100

position = positionClass()
alpha = 0.5

def gradient_ascent(unit_vector):
	position.grad_cont = True
	x, y, Lt = symbols('x y Lt')
	xyz = [0]*3
	new_x = 1
	new_y = 1
	new_z = 1
	J = position.J(unit_vector, position.y())
	djdx = diff(J, x)
	djdy = diff(J, y)
	while position.cont() == True and sqrt((new_x-xyz[0])**2+(new_y-xyz[1])**2+(new_z-xyz[3])**2) > 0.1:
		new_x = position.x() + djdx.subs([(x, position.x()),(y,position.y()), (Lt, 85.3)])*alpha
		new_y = position.y() + djdy.subs([(x,position.x()),(y,position.y()), (Lt, 85.3)])*alpha
		new_z = position.z() #This can't be correct
		position.updateXYZ(new_x,new_y,new_z)
		vector_plot = np.array([[0,0,0,b_x.subs([(x, position.x()),(y,position.y()), (Lt, 85.3)]), b_y.subs([(x, position.x()),(y,position.y()), (Lt, 85.3)]), b_z.subs([(x, position.x()),(y,position.y()), (Lt, 85.3)])],unit_vector])

		X, Y, Z, U, V, W = zip(*vector_plot)
		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')
		ax.quiver(X, Y, Z, U, V, W)
		ax.set_xlim([-1, 1])
		ax.set_ylim([-1, 1])
		ax.set_zlim([-1, 1])
		ax.set_xlabel('x')
		ax.set_ylabel('y')
		ax.set_zlabel('z')
		plt.show()
		plt.hold(True)
		xyz = [position.x(),position.y(),position.z()]
		xyz_msg = Float32MultiArray(data = xyz)
		ort_pub.publish(xyz_msg)

	if position.cont() == True:
		new_x = position.x() + djdx.subs([(x, position.x()),(y,position.y()), (Lt, 85.3)])*0.1
		new_y = position.y() + djdy.subs([(x,position.x()),(y,position.y()), (Lt, 85.3)])*0.1
		new_z = position.z() #This can't be correct
		position.updateXYZ(new_x,new_y,new_z)
		xyz = [position.x(),position.y(),position.z()]
		xyz_msg = Float32MultiArray(data = xyz)
		ort_pub.publish(xyz_msg)		

def ort_callback(msg):
	position.grad_cont = False
	xyz = [0]*3
	magnitude = np.sqrt(((msg.data[0])**2) + ((msg.data[1])**2) + ((msg.data[2])**2))
	unit_vector = [msg.data[0], msg.data[1], msg.data[2]]/magnitude
	gradient_ascent(unit_vector)
	

def waypoint_callback(msg):
	position.arrived()

def pos_callback(msg):
	position.updateXYZ(msg.data[0], msg.data[1], msg.data[2])

def orientation():
	rospy.init_node('grad_calc')
	rospy.Subscriber("/des_ort", Float32MultiArray, ort_callback)
	rospy.Subscriber("/continueWaypoint", Bool, waypoint_callback)
	rospy.Subscriber("/des_pos", Float32MultiArray, pos_callback)
	rospy.spin()

if __name__ == '__main__':
	orientation()


#Need to create plotting section to map out the testing. 