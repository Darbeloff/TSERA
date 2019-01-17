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
		self.b_list = []
		self.j_list = []
		self.grad_cont = True
		self.T_vector = [0,0,0]
		self.b_vector = [0,0,1]
		self.djdx = 0
		self.djdy = 0
	def updateXYZ(self,x,y,z, Lt):
		self.x = x
		self.y = y
		self.z = z
		self.updateB(Lt)
	def updatelist(self, xyz):
		if self.list[0] == 0:
			self.list[0] = xyz
		elif self.list[-1] == 0:
			self.list[self.count] = xyz
	def updatecount(self):
		self.count += 1
	def position(self):
		return self.count
	def calc_dj(self, Lt):
		X = self.x
		Y = self.y
		self.djdx = ((2*(sqrt(3)*Lt + 3*X)*(-1 + X/sqrt(X**2 + Y**2)) + 6*(-X + sqrt(X**2 + Y**2)))/(2* sqrt(Lt^2) * sqrt(-6*Y^2 + 2*( sqrt(3)*Lt + 3*X )*(-X + sqrt(X**2 + Y**2)))))*self.T_vector[0] +\
		(((X + sqrt(X**2 + Y**2))*(2*(sqrt(3)*Lt + 3*X)*(-1 + X/ sqrt(X**2 + Y**2)) + 6*(-X + sqrt(X**2 + Y**2))))/(2* sqrt(Lt**2)*Y* sqrt(-6*Y**2 + 2*( sqrt(3)*Lt + 3*X)*(-X + sqrt(X**2 + Y**2)))) + ((1 + X/ sqrt(X**2 + Y**2))* sqrt(-6*Y**2 + 2*( sqrt(3)*Lt + 3*X)*(-X + sqrt(X**2 + Y**2))))/(sqrt(Lt**2)*Y)) * self.T_vector[1] + \
		(-((2* sqrt(3)*X)/( sqrt(Lt**2)* sqrt(X**2 + Y**2))))*self.T_vector[2]

		self.djdy = ((-12*Y + (2*(sqrt(3)*Lt + 3*X)*Y)/ sqrt(X**2 + Y**2))/(2* sqrt(Lt**2)* Sqrt(-6*Y**2 + 2*( sqrt(3)*Lt + 3*X)*(-X + sqrt(X**2 + Y**2))))) * self.T_vector[0] +\
		((((-12*Y + (2*( sqrt(3)*Lt + 3*X)*Y)/ sqrt(X**2 + Y**2))*(X + sqrt(X**2 + Y**2)))/(2* sqrt(Lt**2)*Y* sqrt(-6*Y**2 + 2*( sqrt(3)*Lt + 3*X)*(-X + sqrt(X**2 + Y**2)))) + sqrt(-6*Y**2 + 2*( sqrt(3)*Lt + 3*X)*(-X + sqrt(X**2 + Y**2)))/( sqrt(Lt**2)* sqrt(X**2 + Y**2)) - ((X + sqrt(X**2 + Y**2))* sqrt(-6*Y**2 + 2*( sqrt(3)*Lt + 3*X)*(-X + sqrt(X**2 + Y**2))))/( sqrt(Lt**2)*Y**2)))*self.T_vector[1] + \
		((-((2* sqrt(3)*Y)/( sqrt(Lt**2)* sqrt(X**2 + Y**2)))))*self.T_vector[2]

		return [djdx, djdy]
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
		self.b_list.append(self.b_vector)
		self.j_list.append([self.x, self.y])
	def b_vec(self):
		return self.b_vector

alpha = 0.5

def J(x,y, unit_vector, Lt):
	b_x = (sqrt((-6*y**2)+2*(sqrt(3)*Lt+3*x)*(-x+sqrt(x**2+y**2))))/(sqrt(Lt**2))
	b_z = (Lt-2*sqrt(3)*sqrt(x**2+y**2))/(sqrt(Lt**2))	

	if x == 0 and y == 0:
		 b_vector = [0,0,1]
	elif abs(y) > 0.1:
		b_y = ((x+sqrt(x**2+y**2))*sqrt(-6*y**2+2*(sqrt(3)*Lt+3*x)*(-x+sqrt(x**2+y**2))))/(sqrt(Lt**2)*y)
		b_vector = [b_x, b_y, b_z]
	else:
		b_vector = [b_x, -b_x*b_z, b_z]

	J = np.dot[b_vector,unit_vector]

	return J
def gradient_ascent(stage, unit_vector):
	if stage == 1:
		pose = pose1
	elif stage == 2:
		pose = pose2
	else:
		pose = pose3
	
	pose.grad_cont = True
	
	xyz = [0]*9
	
	new_x = 1
	new_y = 1
	new_z = 1
	
	while pose.cont() == True and sqrt((new_x-xyz[0])**2+(new_y-xyz[1])**2+(new_z-xyz[3])**2) > 0.001:
		DJ = pose.calc_dj(Lt)
	 	new_x = pose.x + DJ[0]*alpha
	 	new_y = pose.y + Dj[1]*alpha
	 	new_z = pose.z 
	 	pose.updateXYZ(new_x,new_y,new_z, Lt)
		xyz = [pose1.x, pose1.y, pose1.z, pose2.x, pose2.y, pose2.z, pose3.x, pose3.y, pose3.z]
		xyz_msg = Float32MultiArray(data = xyz)
		ort_pub.publish(xyz_msg)	 	


	if pose.cont() == True:
		#vector_plot = np.array([[0,0,0,pose., b_y.subs([(x, position.x),(y,position.y), (Lt, 85.3)]), b_z.subs([(x, position.x),(y,position.y), (Lt, 85.3)])],[0, 0, 0, unit_vector[0], unit_vector[1], unit_vector[2]]])
		# X, Y, Z, U, V, W = zip(*vector_plot)
		 fig = plt.figure(1)
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
		x = []
		y = []
		j = []
		for m in len(pose.j_list):
			x.append(pose.j_list[m][0])
			y.append(pose.j_list[m][1])
			j.append(J(pose.j_list[m][0],pose.j_list[m][1],unit_vector, Lt))


		ax = fig.gca(projection='3d')
		X = np.linspace(0,36.58,100)
		Y = np.linspace(0,24.38,100)
		X, Y = np.meshgrid(X,Y)
		Z = np.zeros((len(X), len(Y)))
		
		for k in range(len(X)):
			for m in range(len(Y)):
				Z[k,m] = J(X[k,m], Y[k,m], unit_vector, Lt)

		ax.plot3D(x,y,j, 'black')
		surf = ax.plot_surface(X,Y,Z,rstride=1,cmap = cm.RdBu, linewidth = 0, antialiased = False)

		fig2 = plt.figure(2)
		plt.contour(X,Y,Z,200)
		plt.plot(x,y)
		plt.show()

def ort_callback(msg):
	T1 = [msg.data[0], msg.data[1], msg.data[2]]
	T2 = [msg.data[3], msg.data[4], msg.data[5]]
	T3 = [msg.data[6], msg.data[7], msg.data[8]]
	print ('T3 = ', T3)
	if (pose1.T()!=T1).any():
		pose1.updateT(T1)
		gradient_ascent(1, T1)
	elif (pose2.T()!=T2).any():
		pose2.updateT(T2)
		gradient_ascent(2, T2)
	elif (pose3.T()!=T3).any():
		pose3.updateT(T3)
		gradient_ascent(3, T3)
	
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
