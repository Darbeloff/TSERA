#!/usr/bin/python
import rospy
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

ort_pub = rospy.Publisher("/des_ort_xyz", Float32MultiArray, queue_size = 1)

class poseClass():
	def __init__(self, stage_):
		self.stage = stage_
		self.x = 0
		self.y = 0
		self.z = 92
		self.count = 0
		self.b_list = [0]*100
		self.grad_cont = True
		self.T_vector = [0,0,0]
		self.b_vector = [0,0,1]
	def updateXYZ(self,x,y,z):
		self.x = x
		self.y = y
		self.z = z
	def updatelist(self, xyz):
		if self.list[0] == 0:
			self.list[0] = xyz
		elif self.list[-1] == 0:
			self.list[self.count] = xyz
	def updatecount(self):
		self.count += 1
	def position(self):
		return self.count
	def J(self, T):
		J = np.dot(self.b_vector, self.T_vector)
		X = self.x
		Y = self.y
		djdx = (0.01504299815501891*(2*(33.238054997246756 + X)*(-1. + X/sqrt(X^2 + Y^2) + 2.*(-1.*X + sqrt(X**2 + Y**2)))/sqrt(-2*Y**2 + 2.*(33.238054997246756 + X)*(-1.*X + sqrt(X**2 + Y**2)) - \
		0.000905167173967609* sqrt(-2*Y**2 + 2*(33.238054997246756 + X)*(-1*X + sqrt(X**2 + Y**2)) +\
		0.000522598511551812* sqrt(-6*Y**2 + 6*(33.238054997246756 + X)*(-1*X + sqrt(X**2 + Y**2))) - \
		0.000905167173967609*sqrt(X**2 + Y**2)*((0.5*(2*(33.238054997246756 + X)*(-1 + X/sqrt(X**2 + Y**2) + 2*(-1*X + sqrt(X**2 + Y**2)))/\
   		sqrt(-2*Y**2 + 2*(33.238054997246756 + X)*(-1*X + sqrt(X**2 + Y**2))) - (0.4082482904638632*(3*(33.238054997246756 + X)*(-1 + X/sqrt(X**2 + Y**2)) + 3*(-1*X + sqrt(X**2 + Y**2))))/\
   		sqrt(-3*Y**2 + 3*(33.238054997246756 + X)*(-1*X + sqrt(X**2 + Y^2)))) - \
   		(1/sqrt(X**2 + Y**2))*0.000905167173967609*X*(1*sqrt(-2*Y**2 + 2*(33.238054997246756 + X)*(-1*X + sqrt(X**2 + Y**2))) - 0.8164965809277264*\
     	sqrt(-3*Y**2 + 3*(33.238054997246756 + X)*(-1*X + sqrt(X**2 + Y**2)))) + \
   		X*(-((0.0004525835869838045*(2*(33.238054997246756 + X)*(-1 + X/sqrt(X**2 + Y**2)) + 2*(-1*X + sqrt(X**2 + Y**2)))/\
       	sqrt(-2*Y**2 + 2*(33.238054997246756 + X)*(-1*X + Sqrt[X**2 + Y**2]))) + \
        (0.000261299255775906*(6*(33.238054997246756 + X)*(-1 + X/sqrt(X**2 + Y**2)) + 6*(-1*X + sqrt(X**2 + Y**2))))/\
        sqrt(-6*Y**2 + 6*(33.238054997246756 + X)*(-1*X + sqrt(X**2 + Y**2)))))))))))
		return J
	def arrived(self):
		self.list.pop(0)
		xyz_msg = Float32MultiArray(data = self.list[0])
		ort_pub.publish(xyz_msg)
	def cont(self):
		return self.grad_cont
	def updateT(self, vector):
		self.b_list.clear()
		self.T_vector = vector
	def T(self):
		return self.T_vector
	def updateB(self, Lt):
		if self.x == 0 and self.y == 0:
			 self.b_vector = [0,0,1]
		elif abs(self.y) > 0.1:
			b_x = (sqrt((-6*self.y**2)+2*(sqrt(3)*Lt+3*self.x)*(-self.x+sqrt(self.x**2+self.y**2))))/(sqrt(Lt**2))
			b_y = ((self.x+sqrt(self.x**2+self.y**2))*sqrt(-6*self.y**2+2*(sqrt(3)*Lt+3*self.x)*(-self.x+sqrt(self.x**2+self.y**2))))/(sqrt(Lt**2)*self.y)
			b_z = (Lt-2*sqrt(3)*sqrt(self.x**2+self.y**2))/(sqrt(Lt**2))
			self.b_vector = [b_x, b_y, b_z]
		else:
			b_x = (sqrt((-6*self.y**2)+2*(sqrt(3)*Lt+3*self.x)*(-self.x+sqrt(self.x**2+self.y**2))))/(sqrt(Lt**2))
			b_z = (Lt-2*sqrt(3)*sqrt(self.x**2+self.y**2))/(sqrt(Lt**2))
			self.b_vector = [b_x, -b_x*b_z, b_z]
		self.list.append(b_vector)
	def b_vec(self):
		return self.b_vector

alpha = 0.5

def gradient_ascent(stage, unit_vector):
	if stage == 1:
		pose = pose1
	elif stage == 2:
		pose = pose2
	else:
		pose = pose3
	
	pose.updateT(unit_vector)
	pose.grad_cont = True
	x, y, Lt = symbols('x y Lt')
	
	xyz = [0]*9
	
	new_x = 1
	new_y = 1
	new_z = 1

	J = pose.J(unit_vector)
	
	while pose.cont() == True and sqrt((new_x-xyz[0])**2+(new_y-xyz[1])**2+(new_z-xyz[3])**2) > 0.1:
	 	new_x = 
	 	new_y = 
	 	new_z = position.z 
	 	position.updateXYZ(new_x,new_y,new_z)
	 	
	 	#vector_plot = np.array([[0,0,0,b_x.subs([(x, position.x),(y,position.y), (Lt, 85.3)]), b_y.subs([(x, position.x),(y,position.y), (Lt, 85.3)]), b_z.subs([(x, position.x),(y,position.y), (Lt, 85.3)])],[0, 0, 0, unit_vector[0], unit_vector[1], unit_vector[2]]])

		# X, Y, Z, U, V, W = zip(*vector_plot)
		# fig = plt.figure()
		# ax = fig.add_subplot(111, projection='3d')
		# ax.quiver(U, V, W, X, Y, Z)
		# ax.set_xlim([-1, 1])
		# ax.set_ylim([-1, 1])
		# ax.set_zlim([-1, 1])
		# ax.set_xlabel('x')
		# ax.set_ylabel('y')
		# ax.set_zlabel('z')
		# plt.show()
		# plt.hold(True)

	xyz = [pose1.x,pose1.y,pose1.z, pose2.x,pose2.y,pose2.z, pose3.x,pose3.y,pose3.z]
	xyz_msg = Float32MultiArray(data = xyz)
	ort_pub.publish(xyz_msg)		

def ort_callback(msg):
	T1 = [msg.data[0], msg.data[1], msg.data[2]]
	T2 = [msg.data[3], msg.data[4], msg.data[5]]
	T3 = [msg.data[6], msg.data[7], msg.data[8]]
	print ('T3 = ', T3)
	if (pose1.T()!=T1).any():
		gradient_ascent(1, T1)
		pose1.updateT(T1)
	elif (pose2.T()!=T2).any():
		gradient_ascent(2, T2)
		pose2.updateT(T2)
	elif (pose3.T()!=T3).any():
		gradient_ascent(3, T3)
		pose3.updateT(T3)
	
# def waypoint_callback(msg):
# 	pos1.arrived()

def pos_callback(msg):
	pose1.updateXYZ(msg.data[0], msg.data[1], msg.data[2])
	pose2.updateXYZ(msg.data[3], msg.data[4], msg.data[5])
	pose3.updateXYZ(msg.data[6], msg.data[7], msg.data[8])

def orientation():
	rospy.init_node('grad_calc')
	rospy.Subscriber("/des_ort", Float32MultiArray, ort_callback)
	#rospy.Subscriber("/continueWaypoint", Bool, waypoint_callback)
	rospy.Subscriber("/des_pos", Float32MultiArray, pos_callback)
	rospy.spin()

if __name__ == '__main__':
	pose1 = poseClass(1)
	pose2 = poseClass(2)
	pose3 = poseClass(3)
	orientation()
