#!/usr/bin/python
import rospy
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np
from matplotlib import cm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import time

ort_pub = rospy.Publisher("/des_ort_xyz", Float32MultiArray, queue_size = 1)
# continue_loop = True

class poseClass():
	def __init__(self, stage_):
		self.stage = stage_
		self.x = 0
		self.y = 0
		self.z = 92
		self.b_list = []
		self.j_list = []
		self.grad_cont = True
		self.T_vector = [0,0,1]
		self.b_vector = [0,0,1]
		self.djdx = 0
		self.djdy = 0
	def updateXYZ(self,x,y,z, Lt):
		self.x = x
		self.y = y
		self.z = z
		self.updateB(Lt)

	def calc_dj(self, Lt):
		X = self.x
		Y = self.y
		djdx1 = (2*(np.sqrt (3)*Lt + 3*X)*(-1 + X/np.sqrt (X ** 2 + Y ** 2)) + 6*(-X + np.sqrt (X ** 2 + Y ** 2)))/ (2*np.sqrt (Lt ** 2)*np.sqrt (-6*Y ** 2 + 2*(np.sqrt (3)*Lt + 3*X)*(-X + np.sqrt (X ** 2 + Y ** 2))))
		djdx3 = (-2*np.sqrt (3)*X)/(np.sqrt (Lt ** 2)*np.sqrt (X ** 2 + Y ** 2))
		djdy1 = (-12*Y + (2*(np.sqrt (3)*Lt + 3*X)*Y)/np.sqrt (X ** 2 + Y ** 2))/(2*np.sqrt (Lt ** 2)*np.sqrt (-6*Y ** 2 + 2*(np.sqrt (3)*Lt + 3*X)*(-X + np.sqrt (X ** 2 + Y ** 2))))
		djdy3 = (-2*np.sqrt (3)*Y)/(np.sqrt (Lt ** 2)*np.sqrt (X ** 2 + Y ** 2))		

		if abs(self.y) >= 0.1:
			djdx2 = ((X + np.sqrt (X ** 2 + Y ** 2))*(2*(np.sqrt (3)*Lt + 3*X)*(-1 + X/np.sqrt (X ** 2 + Y ** 2)) + 6*(-X + np.sqrt (X ** 2 + Y ** 2))))/(2*np.sqrt (Lt ** 2)*Y*np.sqrt (-6*Y ** 2 + 2*(np.sqrt (3)*Lt + 3*X)*(-X + np.sqrt (X ** 2 + Y ** 2)))) + ((1 + X/np.sqrt (X ** 2 + Y ** 2))*np.sqrt (-6*Y ** 2 + 2*(np.sqrt (3)*Lt + 3*X)*(-X + np.sqrt (X ** 2 + Y ** 2))))/(np.sqrt (Lt ** 2)*Y)
			djdy2 = ((-12*Y + (2*(np.sqrt (3)*Lt + 3*X)*Y)/np.sqrt (X ** 2 + Y ** 2))*(X + np.sqrt (X ** 2 + Y ** 2)))/(2*np.sqrt (Lt ** 2)*Y*np.sqrt (-6*Y ** 2 + 2*(np.sqrt (3)*Lt + 3*X)*(-X + np.sqrt (X ** 2 + Y ** 2)))) + np.sqrt (-6*Y ** 2 + 2*(np.sqrt (3)*Lt + 3*X)*(-X + np.sqrt (X ** 2 + Y ** 2)))/(np.sqrt (Lt ** 2)*np.sqrt (X ** 2 + Y ** 2)) - ((X + np.sqrt (X ** 2 + Y ** 2))*np.sqrt (-6*Y ** 2 + 2*(np.sqrt (3)*Lt + 3*X)*(-X + np.sqrt (X ** 2 + Y ** 2))))/(np.sqrt (Lt ** 2)*Y ** 2)
			djdx = djdx1*self.T_vector[0] + djdx2*self.T_vector[1] + djdx3*self.T_vector[2]
			djdy = djdy1*self.T_vector[0] + djdy2*self.T_vector[1] + djdy3*self.T_vector[2]
			self.djdx = djdx
			self.djdy = djdy			
			return [djdx, djdy, 1]

		elif (abs(self.y) <= 0.1) != (self.x>-0.1):
			#derivative where b_y = np.sqrt(1-b_x**2-b_z**2)
			r = np.sqrt(X**2+Y**2)
			a = X + r
			numeratorx = np.sqrt(3)*Lt*a - 3*(2*X**2+Y**2+2*X*r)
			numeratory = Y*(np.sqrt(3)*Lt-3*(X+2*r))
			denom_sqrt = (np.sqrt(3)*Lt*a - 3*((X**2)+(Y**2)+X*r))/(Lt**2)
			denominator = np.sqrt(2)*(Lt**2)*r*np.sqrt(denom_sqrt)

			djdx2 = numeratorx/denominator
			djdy2 = (numeratory/denominator)

			djdx = djdx1*self.T_vector[0] + djdx2*self.T_vector[1] + djdx3*self.T_vector[2]
			djdy = djdy1*self.T_vector[0] + djdy2*self.T_vector[1] + djdy3*self.T_vector[2]
			self.djdx = djdx
			self.djdy = djdy
			return [djdx, djdy, 2, djdy1, djdy2, djdy3]
		else:
			#Not T_vector, but difference between B and T
			r = np.sqrt(self.x**2+self.y**2)
			if self.djdy != 0:
				djdy = np.sign(self.djdy)*np.log10(abs(self.djdy))
				djdx = np.sign(self.djdx)*np.log10(abs(self.djdx))
			else:
				djdy = 1
				djdx = 1
			self.djdy = djdy
			self.djdx = djdx
			return [djdx, djdy, 3]
	def updateT(self, vector):
		if len(self.b_list) > 0:
			del self.b_list[:]
		self.T_vector = vector
	def T(self):
		return self.T_vector
	def updateB(self, Lt):
		b_x = (np.sqrt((-6*self.y**2)+2*(np.sqrt(3)*Lt+3*self.x)*(-self.x+np.sqrt(self.x**2+self.y**2))))/(np.sqrt(Lt**2))
		b_z = (Lt-2*np.sqrt(3)*np.sqrt(self.x**2+self.y**2))/(np.sqrt(Lt**2))
		if self.x == 0 and self.y == 0:
			 self.b_vector = [0,0,1]
		elif abs(self.y) > 0.1:
			b_y = ((self.x+np.sqrt(self.x**2+self.y**2))*np.sqrt(-6*self.y**2+2*(np.sqrt(3)*Lt+3*self.x)*(-self.x+np.sqrt(self.x**2+self.y**2))))/(np.sqrt(Lt**2)*self.y)
			self.b_vector = [b_x, b_y, b_z]
		else:
			self.b_vector = [b_x, np.sqrt(1-b_x**2-b_z**2), b_z]
		self.b_list.append(self.b_vector)
		self.j_list.append([self.x, self.y])
	def b_vec(self):
		return self.b_vector

def J(x,y, unit_vector, Lt):
	b_x = (np.sqrt((-6*y**2)+2*(np.sqrt(3)*Lt+3*x)*(-x+np.sqrt(x**2+y**2))))/(np.sqrt(Lt**2))
	b_z = (Lt-2*np.sqrt(3)*np.sqrt(x**2+y**2))/(np.sqrt(Lt**2))	

	if x == 0 and y == 0:
		 b_vector = [0,0,1]
	elif abs(y) > 0.1:
		b_y = ((x+np.sqrt(x**2+y**2))*np.sqrt(-6*y**2+2*(np.sqrt(3)*Lt+3*x)*(-x+np.sqrt(x**2+y**2))))/(np.sqrt(Lt**2)*y)
		b_vector = [b_x, b_y, b_z]
	else:
		b_vector = [b_x, np.sqrt(1-b_x**2-b_z**2), b_z]

	J = np.dot(b_vector,unit_vector)

	return J
def gradient_ascent(stage, unit_vector):
	if stage == 1:
		pose = pose1
	elif stage == 2:
		pose = pose2
	else:
		pose = pose3
	
	continue_loop = True
	
	xyz = [0]*9
	
	new_x = 1
	new_y = 1
	new_z = 1
	Lt = 85.3 #Need to calculate from CAD
	alpha = 0.1
	prev_dj0 = 0
	prev_dj1 = 0
	count = 0
	past_t = pose.T()[:]
	step = 0
	djy = 0
	djx = 0
	r = rospy.Rate(100)
	print "starting loop"
	while continue_loop == True and np.linalg.norm(np.cross(pose.b_vec(), pose.T())) > 0.0048: #Current loop below can get below .005, which is an error of .865 cm
		if(pose.T() == past_t) == False:
			continue_loop = False
			break
		DJ = pose.calc_dj(Lt)

		if np.sign(djx) != np.sign (DJ[0]) and step > 0 and abs(pose.x) < 0.1:
			new_x = pose.x + djx*alpha
		else:
			new_x = pose.x +DJ[0]*alpha
			djx = DJ[0]
		if np.sign(djy) != np.sign(DJ[1]) and step > 0 and abs(pose.y) < 0.1:
			new_y = pose.y + djy*alpha
		else:
	 		new_y = pose.y + DJ[1]*alpha 
	 		djy = DJ[1]
	 	new_z = pose.z
	 	pose.updateXYZ(new_x,new_y,new_z, Lt)
		xyz = [pose1.x, pose1.y, pose1.z, pose2.x, pose2.y, pose2.z, pose3.x, pose3.y, pose3.z]
		xyz_msg = Float32MultiArray(data = xyz)
		ort_pub.publish(xyz_msg)

		print new_x, new_y, DJ[0], DJ[1], DJ[2] # pose.b_vec()[0], pose.b_vec()[2], np.linalg.norm(np.cross(pose.b_vec(), pose.T()))
		step += 1
		if step > 50000:
			break
		#r.sleep()
	print step

	if continue_loop == True:

		fig = plt.figure(1)
		x = []
		y = []
		j = []
		for m in range(len(pose.j_list)):
			x.append(pose.j_list[m][0])
			y.append(pose.j_list[m][1])
			j.append(J(pose.j_list[m][0],pose.j_list[m][1],unit_vector, Lt))


		ax = fig.gca(projection='3d')
		X = np.linspace(-11, 11,100)
		Y = np.linspace(-11, 11,100)
		X, Y = np.meshgrid(X,Y)
		Z = np.zeros((len(X), len(Y)))
		
		for k in range(len(X)):
			for m in range(len(Y)):
				Z[k,m] = J(X[k,m], Y[k,m], unit_vector, Lt)

		ax.plot3D(x,y,j, 'black')
		surf = ax.plot_surface(X,Y,Z,rstride=1, cmap = cm.RdBu, linewidth = 0, antialiased = False)

		fig2 = plt.figure(2)
		plt.contour(X,Y,Z,200)
		plt.plot(x,y)
		plt.show()

def ort_callback(msg):
	T1 = [msg.data[0], msg.data[1], msg.data[2]]
	T2 = [msg.data[3], msg.data[4], msg.data[5]]
	T3 = [msg.data[6], msg.data[7], msg.data[8]]
	print ('T3 = ', T3)
	if msg.data[9] == 11.0:
		#continue_loop = False
		pose1.updateT(T1)
		gradient_ascent(1, T1)
	elif msg.data[9] == 10.0:
		#continue_loop = False
		pose2.updateT(T2)
		gradient_ascent(2, T2)
	elif msg.data[9] == 8.0:
		#continue_loop = False
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
