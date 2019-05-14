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
		self.t_list = [0]*5
		self.t_xyz_list = [0]*5
		self.xyz_grad_list = [0]*5
		self.xyz_send_list = [0]*5
		self.grad_cont = True
		self.T_vector = [0,0,1]
		self.T_rot = [0,0,1]
		self.b_vector = [0,0,1]
		self.djdx = 0
		self.djdy = 0
		self.step = 0
		self.send = False
		self.step_calc = 0
		self.wait = True
		self.rotated = 0
		self.quit_loop = False
	def updateXYZ(self,x,y,z, Lt, rotated):
		# if rotated == 0:
		self.x = x
		self.y = y
		self.z = z
			
		# else:
		# 	r = np.sqrt(x**2 + y**2)
		# 	theta = np.arctan(y/x)
		# 	self.x = r*np.cos(theta-rotated*120*np.pi/180)
		# 	self.y = r*np.sin(theta-rotated*120*np.pi/180)
		self.updateB(self.x, self.y, Lt)	

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
			self.t_list = [0]*5
			self.step = 0
			self.xyz_grad_list = [0]*5
			self.xyz_send_list = [0]*5
			self.step_calc = 0
			self.rotated = 0
		value = np.linalg.norm(np.cross(self.b_vector,self.T_vector))
		theta = np.arcsin(min(value, 1))
		print np.cross(self.b_vector,self.T_vector)
		print np.linalg.norm(np.cross(self.b_vector,self.T_vector))
		for i in range(len(self.t_list)):
			scale = np.sin((theta*(i+1)/5))/np.sin(theta)
			self.t_list[i] = [self.b_vector[0]+ scale*(self.T_vector[0]-self.b_vector[0]), self.b_vector[1]+ scale*(self.T_vector[1]-self.b_vector[1]), self.b_vector[2]+scale*(self.T_vector[2]-self.b_vector[2])] 
			xyz_grad, xyz_send = gradient_ascent(self.stage, self.t_list[i], i)
			self.xyz_grad_list[i] = xyz_grad
			self.xyz_send_list[i] = xyz_send
			self.step_calc += 1
			if self.T_vector != vector:
				self.wait = True
				break
			if self.quit_loop == True:
				err_msg = "Gradient Ascent failed on step_calc = " + str(self.step_calc)
				error_pub.publish(err_msg)
				break
	def update_step(self):
		self.send = False
		if self.quit_loop == True:
			#Gradient ascent and rotation didn't work
			if self.step == self.step_calc-1:
				self.wait = True
				self.send = True
				err_msg = "Robot has reached final calculated step."
				error_pub.publish(err_msg)
		elif self.step < 4 and self.xyz_send_list[self.step+1] != 0: # and self.step_list[self.step+1] == 1:
			self.step += 1
			if self.xyz_send_list[self.step] != "failed":
				print self.step
				xyz = self.xyz_send_list[self.step]
				xyz_msg = Float32MultiArray(data = xyz)
				ort_pub.publish(xyz_msg)
			else:
				self.wait = True
				#a bit redundant, but fail safe in case somehow we get to next wp but its a failed one 
		else:
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
def T_rotated(T_vector, rotated):
	if T_vector[0] == 0:
		theta = 90
	else:
		theta = np.arctan2(T_vector[1],T_vector[0])
	r = np.sqrt(T_vector[0]**2 + T_vector[1]**2)
	T_rot = [0,0,0]
	T_rot[0] = r*np.cos(theta+rotated*120*np.pi/180)
	T_rot[1] = r*np.sin(theta+rotated*120*np.pi/180)
	T_rot[2] = T_vector[2]
	return T_rot
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

def graph_check(stage, rotate):
	plt.close('all')
	if stage == 1:
		pose = pose1
	elif stage == 2:
		pose = pose2
	else:
		pose = pose3


	if rotate != 0:
		unit_vector = T_rotated(pose.T(), rotate)
	else:
		unit_vector = pose.T()

	fig = plt.figure(1)
	x = []
	y = []
	j = []
	for m in range(len(pose.j_list)):
		r = np.sqrt(pose.j_list[m][0]**2 + pose.j_list[m][1]**2)
		theta = np.arctan(pose.j_list[m][1]/pose.j_list[m][0])

		ex = r*np.cos(theta + rotate*120*np.pi/180)
		ey = r*np.sin(theta + rotate*120*np.pi/180)
		x.append(ex)
		y.append(ey)
		j.append(J(ex,ey,unit_vector, Lt))

	print x, y, j


	ax = fig.gca(projection='3d')
	X = np.linspace(-11, 11,100)
	Y = np.linspace(-11, 11,100)
	X, Y = np.meshgrid(X,Y)
	Z = np.zeros((len(X), len(Y)))
	for k in range(len(X)):
		for m in range(len(Y)):
			Z[k,m] = J(X[k,m], Y[k,m], unit_vector, Lt)
	ax.plot3D(x,y,j, 'black', label = "TSERA Path")
	surf = ax.plot_surface(X,Y,Z,rstride=1, cmap = cm.RdBu, linewidth = 0, antialiased = False)
	title = "Cost Surface for T = " + str(unit_vector)
	ax.text2D(0.05, 0.95, title, transform=ax.transAxes)
	#ax.title("Cost Surface for T = [0.579, 0.579, 0.573")
	ax.set_xlabel("X [cm]")
	ax.set_ylabel("Y [cm]")
	ax.set_zlabel("Z [cm]")
	ax.legend()
	plt.show()

	fig2 = plt.figure(2)
	plt.contour(X,Y,Z,200)
	plt.title(title)
	plt.xlabel("X [cm]")
	plt.ylabel("Y [cm]")
	plt.plot(x,y, label = "TSERA Path")
	plt.legend()
	plt.show()

def gradient_ascent(stage, unit_vector, i):
	if stage == 1:
		pose = pose1
	elif stage == 2:
		pose = pose2
	else:
		pose = pose3
	if i == 0:
		pose.rotated = 0
	
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
	achieved = False
	count_switch = 0

	r = rospy.Rate(100)
	print "starting loop"
	if pose.rotated == 0:
		while continue_loop == True and cross > 0.0048:
			if (pose.T() == past_t) == False:
				print "Vector Changed"
				continue_loop = False
				break
			DJ = pose.calc_dj(currentx, currenty, Lt, unit_vector)
			# if np.sign(djx) != np.sign(DJ[0]) and step > 0 and abs(currentx) < 0.1:
			# 	new_x = currentx + djx*alpha
			# 	count_switch += 1
			# else:
			if np.sign(djx) != np.sign(DJ[0]) and step > 0:
				count_switch += 1
			new_x = currentx +DJ[0]*alpha
			djx = DJ[0]
			if np.sign(djy) != np.sign(DJ[1]) and step > 0 and abs(currenty) < 0.1:
				new_y = currenty + djy*alpha
				count_switch += 1
			if np.sign(djy) != np.sign(DJ[1]) and step > 0:
				count_switch += 1
		 	new_y = currenty + DJ[1]*alpha 
		 	djy = DJ[1]
		 	new_z = currentz
			step += 1
			currentx = new_x
			currenty = new_y
			currentz = new_z

			cross = crossb(currentx, currenty, unit_vector, Lt)
			print step, i, currentx, DJ[0], currenty, DJ[1], count_switch
			#Publishes every cycle of the gradient ascent to get the robot moving while 


			#if x is small, then rotate and do gradient ascent again. perhaps in wp_setup
			if (step > 20000 or count_switch > 20) and i > 0:
			# if step > 20000 and i > 0:
				print "im broken"
				continue_loop = False
				rotate = 1
				pose.rotated = 1
				print step, count_switch, i
				break
				
			elif (step > 20000 or count_switch > 20) and i == 0:
				print "first point didn't converge"

				continue_loop = False
				break
			#r.sleep()
		if continue_loop == True:
			pose.updateXYZ(currentx, currenty, currentz, Lt, rotate)

	if pose.rotated > 0:
		count = 0
		count_loop = 0

		while count < 4 and count_loop < 2 and achieved == False:
		#run while loop again by multiplying T, xyz, and b by rotation matrix
		#send xyz with a 1 at the end of the list for rotation 
		#if close to 0, rotation will still be in the small radius near 0
			print step, i, pose.x, pose.y, pose.z, "before rotation"
			continue_loop = True
			failed = False
			if pose.x == 0:
				theta = 0
			else:
				theta = np.arctan2(pose.y,pose.x)
			r = np.sqrt(pose.x**2 + pose.y**2)
			print theta, r, pose.rotated
			currentx = r*np.cos(theta+pose.rotated*120*np.pi/180)
			currenty = r*np.sin(theta+pose.rotated*120*np.pi/180)
			unit_vector_rot = T_rotated(unit_vector, pose.rotated)
			# currentx = pose.x*-0.5+pose.y*np.sin(120/180*np.pi)
			# currenty = -pose.x*np.sin(120/180*np.pi)+pose.y*0.5
			# currentz = pose.z
			print step, i, currentx, currenty, currentz, "after rotation"
			#need to rotate T
			#need to check if x, y, and z rotation correct
			new_x = 1
			new_y = 1
			new_z = 1
			alpha = 1
			past_t = pose.T()[:]
			cross = 1
			step = 0
			djy = 0
			djx = 0
			count_switch = 0
			print "rotated"
			r = rospy.Rate(100)
			while continue_loop == True and cross > 0.0048:

				if (pose.T() == past_t) == False:
					"I broke the loop"
					continue_loop = False
					break
				DJ = pose.calc_dj(currentx, currenty, Lt, unit_vector_rot)
				# if np.sign(djx) != np.sign (DJ[0]) and step > 0 and abs(currentx) < 0.1:
				# 	new_x = currentx + djx*alpha
				# 	count_switch += 1
				# else:
				if np.sign(djx) != np.sign (DJ[0]) and step > 0:
					count_switch += 1
				new_x = currentx +DJ[0]*alpha
				djx = DJ[0]
				if np.sign(djy) != np.sign(DJ[1]) and step > 0 and abs(currenty) < 0.1:
					new_y = currenty + djy*alpha
			 	else:
			 		new_y = currenty + DJ[1]*alpha
				# else:
				if np.sign(djy) != np.sign (DJ[1]) and step > 0:
					count_switch += 1
 
			 	if count_switch < 5:
			 		djy = DJ[1]
			 	new_z = currentz
				step += 1
				currentx = new_x
				currenty = new_y
				currentz = new_z
				cross = crossb(currentx, currenty, unit_vector_rot, Lt)
				print step, i, currentx, currenty, currentz, pose.rotated
				#if x is small, then rotate and do gradient ascent again. perhaps in wp_setup
				if step > 30000 or count_switch > 20:
					print "im broken"
					continue_loop = False
					failed = True
					achieved = False
					pose.rotated += 1
					# if pose.rotated < 3:
					# 	if count == 0:
					# 		pose.rotated += -1**(pose.rotated)*-1
					# 	elif count == 1:
					# 		pose.rotated = 0
					# 	elif count == 2:
					# 		pose.rotated = 3
					# else:
					# 	pose.rotated = 1
					# 	count = -1
					# 	count_loop += 1
					break
				#r.sleep()
			if failed == False:
				achieved = True
			count += 1
					# return "failed"
				
				
	#publishes message once convergence on final goal
	#if failed == False:
	theta = np.arctan2(currenty,currentx)
	if pose.rotated > 0:
		if pose.x == 0:
			theta = 0
		r = np.sqrt(currentx**2 + currenty**2)
		currentx = r*np.cos(theta-pose.rotated*120*np.pi/180)
		currenty = r*np.sin(theta-pose.rotated*120*np.pi/180)
	if failed == False:
		pose.updateXYZ(currentx, currenty, currentz, Lt, pose.rotated)	
	xyz_grad = [pose1.x, pose1.y, pose1.z, pose1.rotated, pose2.x, pose2.y, pose2.z, pose2.rotated, pose3.x, pose3.y, pose3.z, pose3.rotated]
	rotated =0
	if failed == False:
		if np.sign(pose.T()[0]) == -1:
			theta = np.arctan2(currenty,currentx) +np.pi #- 60/180*np.pi	
			rotated = 0
		r = np.sqrt(currentx**2 + currenty**2)
		currentx = r*np.cos(theta-pose.rotated*120*np.pi/180)
		currenty = r*np.sin(theta-pose.rotated*120*np.pi/180)
	xyz_send = [pose1.x, pose1.y, pose1.z, pose1.rotated, pose2.x, pose2.y, pose2.z, pose2.rotated, currentx, currenty, pose3.z, rotated]
	if i == 0 or pose.send == True:	
		xyz_msg = Float32MultiArray(data = xyz_send)
		ort_pub.publish(xyz_msg)
	# else:
	# 	pose.quit_loop = True
	# 	xyz = "failed"
	# if i == 9:
	#if rotate == 1:
	#graph_check(stage, pose.rotated)
	return xyz_grad, xyz_send

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
