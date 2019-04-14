#!/usr/bin/python
import rospy
from std_msgs.msg import Float32MultiArray, Bool, String
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import cm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
new_one = 0

ort_pub = rospy.Publisher("/des_ort_xyz", Float32MultiArray, queue_size = 1)
error_pub = rospy.Publisher("/error_topic", String, queue_size = 1)
Lt = 85.3
class poseClass():
	def __init__(self, stage_):
		self.stage = stage_
		self.x = 0
		self.y = 0
		self.z = 120
		self.b_list = []
		self.j_list = []
		self.t_list = [0]*10
		self.t_xyz_list = [0]*10
		self.xyz_list = [0]*10
		self.grad_cont = True
		self.T_vector = [0,0,1]
		self.b_vector = [0,0,1]
		self.djdx = 0
		self.djdy = 0
		self.step = 0
		self.send = False
		self.step_calc = 0
		self.wait = True
		self.rotated = False
		self.quit_loop = False
	def updateXYZ(self,x,y,z, Lt):
		self.x = x
		self.y = y
		self.z = z
		self.updateB(x, y, Lt)

	def calc_dj(self, X, Y, Lt, T_vector):
		djdx1 = (2*(np.sqrt (3)*Lt + 3*X)*(-1 + X/np.sqrt (X ** 2 + Y ** 2)) + 6*(-X + np.sqrt (X ** 2 + Y ** 2)))/ (2*np.sqrt (Lt ** 2)*np.sqrt (-6*Y ** 2 + 2*(np.sqrt (3)*Lt + 3*X)*(-X + np.sqrt (X ** 2 + Y ** 2))))
		djdx3 = (-2*np.sqrt (3)*X)/(np.sqrt (Lt ** 2)*np.sqrt (X ** 2 + Y ** 2))
		djdy1 = (-12*Y + (2*(np.sqrt (3)*Lt + 3*X)*Y)/np.sqrt (X ** 2 + Y ** 2))/(2*np.sqrt (Lt ** 2)*np.sqrt (-6*Y ** 2 + 2*(np.sqrt (3)*Lt + 3*X)*(-X + np.sqrt (X ** 2 + Y ** 2))))
		djdy3 = (-2*np.sqrt (3)*Y)/(np.sqrt (Lt ** 2)*np.sqrt (X ** 2 + Y ** 2))		

		if abs(Y) >= 0.1:
			djdx2 = ((X + np.sqrt (X ** 2 + Y ** 2))*(2*(np.sqrt (3)*Lt + 3*X)*(-1 + X/np.sqrt (X ** 2 + Y ** 2)) + 6*(-X + np.sqrt (X ** 2 + Y ** 2))))/(2*np.sqrt (Lt ** 2)*Y*np.sqrt (-6*Y ** 2 + 2*(np.sqrt (3)*Lt + 3*X)*(-X + np.sqrt (X ** 2 + Y ** 2)))) + ((1 + X/np.sqrt (X ** 2 + Y ** 2))*np.sqrt (-6*Y ** 2 + 2*(np.sqrt (3)*Lt + 3*X)*(-X + np.sqrt (X ** 2 + Y ** 2))))/(np.sqrt (Lt ** 2)*Y)
			djdy2 = ((-12*Y + (2*(np.sqrt (3)*Lt + 3*X)*Y)/np.sqrt (X ** 2 + Y ** 2))*(X + np.sqrt (X ** 2 + Y ** 2)))/(2*np.sqrt (Lt ** 2)*Y*np.sqrt (-6*Y ** 2 + 2*(np.sqrt (3)*Lt + 3*X)*(-X + np.sqrt (X ** 2 + Y ** 2)))) + np.sqrt (-6*Y ** 2 + 2*(np.sqrt (3)*Lt + 3*X)*(-X + np.sqrt (X ** 2 + Y ** 2)))/(np.sqrt (Lt ** 2)*np.sqrt (X ** 2 + Y ** 2)) - ((X + np.sqrt (X ** 2 + Y ** 2))*np.sqrt (-6*Y ** 2 + 2*(np.sqrt (3)*Lt + 3*X)*(-X + np.sqrt (X ** 2 + Y ** 2))))/(np.sqrt (Lt ** 2)*Y ** 2)
			djdx = djdx1*T_vector[0] + djdx2*T_vector[1] + djdx3*T_vector[2]
			djdy = djdy1*T_vector[0] + djdy2*T_vector[1] + djdy3*T_vector[2]
			self.djdx = djdx
			self.djdy = djdy			
			return [djdx, djdy, 1]

		elif (abs(Y) <= 0.1) != (X>-0.1):
			#derivative where b_y = np.sqrt(1-b_x**2-b_z**2)
			r = np.sqrt(X**2+Y**2)
			a = X + r
			numeratorx = np.sqrt(3)*Lt*a - 3*(2*X**2+Y**2+2*X*r)
			numeratory = Y*(np.sqrt(3)*Lt-3*(X+2*r))
			denom_sqrt = (np.sqrt(3)*Lt*a - 3*((X**2)+(Y**2)+X*r))/(Lt**2)
			denominator = np.sqrt(2)*(Lt**2)*r*np.sqrt(denom_sqrt)

			djdx2 = numeratorx/denominator
			djdy2 = (numeratory/denominator)

			djdx = djdx1*T_vector[0] + djdx2*T_vector[1] + djdx3*T_vector[2]
			djdy = djdy1*T_vector[0] + djdy2*T_vector[1] + djdy3*T_vector[2]
			self.djdx = djdx
			self.djdy = djdy
			return [djdx, djdy, 2, djdy1, djdy2, djdy3]
		else:
			#Not T_vector, but difference between B and T
			r = np.sqrt(X**2+Y**2)
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
		print "im updating"
		self.T_vector = vector
		self.wait = False
		print "ive changed!"
		if len(self.b_list) > 0 or len(self.j_list) > 0:
			# del self.b_list[:]
			# del self.j_list[:]
			self.t_list = [0]*10
			self.step = 0
			self.xyz_list = [0]*10
			self.step_calc = 0
		theta = np.arcsin(np.linalg.norm(np.cross(self.b_vector,self.T_vector)))
		for i in range(len(self.t_list)):
			scale = np.sin((theta*(i+1)/10))/np.sin(theta)
			self.t_list[i] = [self.b_vector[0]+ scale*(self.T_vector[0]-self.b_vector[0]), self.b_vector[1]+ scale*(self.T_vector[1]-self.b_vector[1]), self.b_vector[2]+scale*(self.T_vector[2]-self.b_vector[2])] 
			xyz = gradient_ascent(self.stage, self.t_list[i], i)
			self.xyz_list[i] = xyz
			self.step_calc += 1
			if self.T_vector != vector:
				self.wait = True
				break
			if self.quit_loop == True:
				err_msg = "Gradient Ascent failed on step_calc = " + self.step_calc
				error_pub.publish(err_msg)
				break

	def update_step(self):
		# try:
		# 	step = 10 - self.step_list[::-1].index(1)
		# except ValueError: 
		# 	step = 0
		# if step <= self.step:
		# 	self.first = True
		
		if self.quit_loop == True:
			#Gradient ascent and rotation didn't work
			if self.step == self.step_calc-1:
				self.step = 0
				self.wait = True
				err_msg = "Robot has reached final calculated step."
				error_pub.publish(err_msg)

		elif self.step == self.step_calc:
			#robot got to waypoint before the next one is ready
			self.send = True
			self.step += 1		

		elif self.step < 9 and self.wait == False: # and self.step_list[self.step+1] == 1:
			self.step += 1
			if self.xyz_list[self.step] != "failed":
				print self.step
				xyz = self.xyz_list[self.step]
				xyz_msg = Float32MultiArray(data = xyz)
				ort_pub.publish(xyz_msg)
			else:
				self.step = 0
				self.wait = True
				#a bit redundant, but fail safe in case somehow we get to next wp but its a failed one 
		else:
			self.step = 0
			self.wait = True

		#instead of publish and then run the next step, run all the steps and just send the next point once it reaches
	
	def T(self):
		return self.T_vector
	def checkT(self, vector):
		if self.T_vector != vector:
			self.T_vector = vector
	def updateB(self, X, Y, Lt):
		b_x = (np.sqrt((-6*Y**2)+2*(np.sqrt(3)*Lt+3*X)*(-X+np.sqrt(X**2+Y**2))))/(np.sqrt(Lt**2))
		b_z = (Lt-2*np.sqrt(3)*np.sqrt(X**2+Y**2))/(np.sqrt(Lt**2))
		if X == 0 and Y == 0:
			 self.b_vector = [0,0,1]
		elif abs(Y) > 0.1:
			b_y = ((X+np.sqrt(X**2+Y**2))*np.sqrt(-6*Y**2+2*(np.sqrt(3)*Lt+3*X)*(-X+np.sqrt(X**2+Y**2))))/(np.sqrt(Lt**2)*Y)
			self.b_vector = [b_x, b_y, b_z]
		else:
			self.b_vector = [b_x, np.sqrt(1-b_x**2-b_z**2), b_z]
		self.b_list.append(self.b_vector)
		self.j_list.append([X, Y])
	def b_vec(self):
		return self.b_vector

def crossb(x, y, unit_vector, Lt):
	b_x = (np.sqrt((-6*y**2)+2*(np.sqrt(3)*Lt+3*x)*(-x+np.sqrt(x**2+y**2))))/(np.sqrt(Lt**2))
	b_z = (Lt-2*np.sqrt(3)*np.sqrt(x**2+y**2))/(np.sqrt(Lt**2))	

	if x == 0 and y == 0:
		 b_vector = [0,0,1]
	elif abs(y) > 0.1:
		b_y = ((x+np.sqrt(x**2+y**2))*np.sqrt(-6*y**2+2*(np.sqrt(3)*Lt+3*x)*(-x+np.sqrt(x**2+y**2))))/(np.sqrt(Lt**2)*y)
		b_vector = [b_x, b_y, b_z]
	else:
		b_vector = [b_x, np.sqrt(1-b_x**2-b_z**2), b_z]	

	return np.linalg.norm(np.cross(b_vector, unit_vector))

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

def graph_check(stage):
	plt.close('all')
	if stage == 1:
		pose = pose1
	elif stage == 2:
		pose = pose2
	else:
		pose = pose3

	fig = plt.figure(1)
	x = []
	y = []
	j = []
	for m in range(len(pose.j_list)):
		x.append(pose.j_list[m][0])
		y.append(pose.j_list[m][1])
		j.append(J(pose.j_list[m][0],pose.j_list[m][1],pose.T(), Lt))

	print x, y, j

	ax = fig.gca(projection='3d')
	X = np.linspace(-11, 11,100)
	Y = np.linspace(-11, 11,100)
	X, Y = np.meshgrid(X,Y)
	Z = np.zeros((len(X), len(Y)))
	for k in range(len(X)):
		for m in range(len(Y)):
			Z[k,m] = J(X[k,m], Y[k,m], pose.T(), Lt)
	ax.plot3D(x,y,j, 'black')
	surf = ax.plot_surface(X,Y,Z,rstride=1, cmap = cm.RdBu, linewidth = 0, antialiased = False)
	plt.show()

	# fig2 = plt.figure(2)
	# plt.contour(X,Y,Z,200)
	# plt.plot(x,y)
	# plt.show()

def gradient_ascent(stage, unit_vector, i):
	if stage == 1:
		pose = pose1
	elif stage == 2:
		pose = pose2
	else:
		pose = pose3
	
	continue_loop = True
	rotate = 0
	failed = False
	currentx = pose.x
	currenty = pose.y
	currentz = pose.z
	new_x = 1
	new_y = 1
	new_z = 1
	alpha = 1
	cross = 1
	past_t = pose.T()[:]
	step = 0
	djy = 0
	djx = 0

	r = rospy.Rate(100)
	print "starting loop"
	while continue_loop == True and cross > 0.0048:
		if (pose.T() == past_t) == False:
			print "Vector Changed"
			continue_loop = False
			break
		DJ = pose.calc_dj(currentx, currenty, Lt, unit_vector)
		if np.sign(djx) != np.sign (DJ[0]) and step > 0 and abs(currentx) < 0.1:
			new_x = currentx + djx*alpha
		else:
			new_x = currentx +DJ[0]*alpha
			djx = DJ[0]
		if np.sign(djy) != np.sign(DJ[1]) and step > 0 and abs(currenty) < 0.1:
			new_y = currenty + djy*alpha
		else:
	 		new_y = currenty + DJ[1]*alpha 
	 		djy = DJ[1]
	 	new_z = currentz
		step += 1
		currentx = new_x
		currenty = new_y
		currentz = new_z
		cross = crossb(currentx, currenty, unit_vector, Lt)
		print step, i, currentx, currenty, currentz
		#Publishes every cycle of the gradient ascent to get the robot moving while 


		#if x is small, then rotate and do gradient ascent again. perhaps in wp_setup
		if step > 50000:
			print "im broken"
			continue_loop = False
			rotate = 1
			break
			# return "failed"
		#r.sleep()
	if continue_loop == True:
		pose.updateXYZ(currentx, currenty, currentz, Lt)

	elif rotate == 1:
		#run while loop again by multiplying T, xyz, and b by rotation matrix
		#send xyz with a 1 at the end of the list for rotation 
		#if close to 0, rotation will still be in the small radius near 0
		print step, i, pose.x, pose.y, pose.z, "before rotation"
		continue_loop = True
		theta = np.arctan(pose.y/pose.x)
		r = np.sqrt(pose.x**2 + pose.y**2)
		currentx = r*np.cos(theta +120)
		currenty = r*np.sin(theta+120)
		# currentx = pose.x*-0.5+pose.y*np.sin(120/180*np.pi)
		# currenty = -pose.x*np.sin(120/180*np.pi)+pose.y*0.5
		# currentz = pose.z
		print step, i, currentx, currenty, currentz, "after rotation"
		#need to rotate T
		#need to check if x, y, and z rotation correct
		new_x = 1
		new_y = 1
		new_z = 1
		alpha = 0.1
		past_t = pose.T()[:]
		cross = 1
		step = 0
		djy = 0
		djx = 0
		print "rotated"
		while continue_loop == True and cross > 0.0048:
			if (pose.T() == past_t) == False:
				"I broke the loop"
				continue_loop = False
				break
			DJ = pose.calc_dj(currentx, currenty, Lt, unit_vector)
			if np.sign(djx) != np.sign (DJ[0]) and step > 0 and abs(currentx) < 0.1:
				new_x = currentx + djx*alpha
			else:
				new_x = currentx +DJ[0]*alpha
				djx = DJ[0]
			if np.sign(djy) != np.sign(DJ[1]) and step > 0 and abs(currenty) < 0.1:
				new_y = currenty + djy*alpha
			else:
		 		new_y = currenty + DJ[1]*alpha 
		 		djy = DJ[1]
		 	new_z = currentz
			step += 1
			currentx = new_x
			currenty = new_y
			currentz = new_z
			cross = crossb(currentx, currenty, unit_vector, Lt)
			print step, i, currentx, currenty, currentz
			#if x is small, then rotate and do gradient ascent again. perhaps in wp_setup
			xyz = [pose1.x, pose1.y, pose1.z, pose2.x, pose2.y, pose2.z, pose3.x, pose3.y, pose3.z, rotate]
			xyz_msg = Float32MultiArray(data = xyz)
			ort_pub.publish(xyz_msg)
			if step > 50000:
				print "im broken"
				continue_loop = False
				failed = True
				break
				# return "failed"
		if failed == False:
			pose.updateXYZ(currentx, currenty, currentz, Lt)
			
	#publishes message once convergence on final goal
	if failed == False:
		xyz = [pose1.x, pose1.y, pose1.z, pose1.rotated, pose2.x, pose2.y, pose2.z, pose2.rotated, pose3.x, pose3.y, pose3.z, pose3.rotated]
		if i == 0 or pose.send == True:
			xyz_msg = Float32MultiArray(data = xyz)
			ort_pub.publish(xyz_msg)
	else:
		pose.quit_loop = True
		xyz = "failed"
	return xyz

def ort_callback(msg):
	print "ort T received"
	T1 = [msg.data[0], msg.data[1], msg.data[2]]
	T2 = [msg.data[3], msg.data[4], msg.data[5]]
	T3 = [msg.data[6], msg.data[7], msg.data[8]]
	if msg.data[9] == 11.0:
		pose1.updateT(T1)
	elif msg.data[9] == 10.0:
		pose2.updateT(T2)
	elif msg.data[9] == 8.0:
		print "ort T sent"
		pose3.updateT(T3)		
	
def pos_callback(msg):
	pose1.updateXYZ(msg.data[0], msg.data[1], msg.data[2])
	pose2.updateXYZ(msg.data[3], msg.data[4], msg.data[5])
	pose3.updateXYZ(msg.data[6], msg.data[7], msg.data[8])

def vec_callback(msg):
	print "vec T received"
	T1 = [msg.data[0], msg.data[1], msg.data[2]]
	T2 = [msg.data[3], msg.data[4], msg.data[5]]
	T3 = [msg.data[6], msg.data[7], msg.data[8]]
	if msg.data[9] == 11.0 and pose1.T() != T1:
		pose1.checkT(T1)
	elif msg.data[9] == 10.0 and pose2.T() != T2:
		pose2.checkT(T2)
	elif msg.data[9] == 8.0 and pose3.T() != T3:
		print "vec T sent"
		pose3.checkT(T3)		

def waypoint_callback(msg):
	#for testing, receiving boolean
	if msg.data == True:
	 	pose1.update_step()

	 	pose2.update_step()

	 	pose3.update_step()

	#If error comes as a float32MultiArray, use this. Check that each motor is less than 0.7
	# if all(i < 0.07 for i in msg.data):

	# 	pose1.update_step()

	# 	pose2.update_step()

	# 	pose3.update_step()

def orientation():
	rospy.init_node('grad_calc')
	rospy.Subscriber("/des_ort", Float32MultiArray, ort_callback)
	rospy.Subscriber("/new_vector", Float32MultiArray, vec_callback)
	rospy.Subscriber("/continueWaypoint", Bool, waypoint_callback)
	rospy.Subscriber("/des_pos", Float32MultiArray, pos_callback)
	rospy.spin()

if __name__ == '__main__':
	pose1 = poseClass(1)
	pose2 = poseClass(2)
	pose3 = poseClass(3)
	orientation()
