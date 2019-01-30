#!/usr/bin/python

import numpy as np


def L_a( x, y, z, Lt, s0, r):
	t1a = -Lt/np.sqrt(3) + r + s0 - 2*x
	t2a = np.sqrt(2/3.0*(r-x)*(np.sqrt(3)*Lt + 3*x) - 2*y**2 ) - z
	la = np.sqrt( t1a**2 + t2a**2)
	return la

def L_b(x, y, z, Lt, s0, r, M, k):

	
	t1b = r + s0 + x - (Lt+3*y)/np.sqrt(3)
	t2b = 1.0/(6.0*y) * (-3*M*r - 3*M*x + np.sqrt(6)*k*y + 6*y*z)
	lb = np.sqrt( t1b**2.0 + t2b**2.0 ) 
	return lb


def L_c(x, y, z, Lt, s0, r, M, k):
	
	t1c = r + s0 + x - (Lt-3*y)/np.sqrt(3)
	t2c = 1.0/(6.0*y) * (3.0*M*r + 3.0*M*x + np.sqrt(6.0)*k*y + 6.0*y*z)
	lc = np.sqrt( t1c**2.0 + t2c**2.0 ) 
	return lc




def constrain_leg_width(rotations):
	rot_min = 0; #revolutions
	rot_max = 43.13/4.0 #revolutions
	if rotations >= rot_max:
		return rot_max
	elif rotations<=rot_min:
		return rot_min
	else:
		return rotations

lo = 30
le = 42.5  + 85 + 42.5+ 30 # sum of all scissor lengths (minus offset)
lb = 4 # length of offset

def width(L_model):
	ball_joint_offset =15 #mm measured
	L = np.sqrt(L_model**2-ball_joint_offset**2)


	#Lmax = 175.33
	#Lmin = 91.433

	w = 2/(lb**2+le**2)*( (lb**2 + lo*le)*np.sqrt(lb**2+le**2-L**2) - L*lb*(le-lo) ) #mm

	#convert to revolutions of the motor shaft
	lead = (1.0/20)*25.4 #mm/revolution of the motor output shaft
	gap = 67.47 #mm from CAD
	hinge_hole_to_outer_edge = 8.54 ## mm measured value
	width_max = gap - 2*(hinge_hole_to_outer_edge) 
	#width_min = 23 #mm measured value
	revolutions = (width_max -  w)/(2*lead)
	
	return constrain_leg_width(revolutions)




def ik_legs(x,y,z):
	'''
	takes x,y,z coordinate in segment base frame and 
	calculates required leg lengths

	'''
	Lt = 85.3 #mm fill in new values with spherical joint
	s0 = 33.23 #mm

	r = np.sqrt(x**2 + y**2)
	la_L = L_a( x,y, z, Lt, s0, r)

	#Avoid singularity
	if(abs(y)<0.001 and abs(x)<0.001):
		lb_L = la_L
		lc_L = la_L
	elif(abs(y)<0.01):
		y = 0.01
		k = np.sqrt(-3.0*y**2.0 + (np.sqrt(3)*Lt + 3*x)*(-x + r))
		M = np.sqrt(2*(r - x)*(np.sqrt(3)*Lt + 3*x) - 6.0*y**2.0)
		lb_L = L_b(x, y, z, Lt, s0, r, M, k)
		lc_L = L_c(x, y, z, Lt, s0, r, M, k)
	else:
		k = np.sqrt(-3*y**2 + (np.sqrt(3)*Lt + 3*x)*(-x + r))
		M = np.sqrt(2*(r - x)*(np.sqrt(3)*Lt + 3*x) - 6*y**2)
		lb_L = L_b(x, y, z, Lt, s0, r, M, k)
		lc_L = L_c(x, y, z, Lt, s0, r, M, k)
	print (la_L, lb_L, lc_L)
	legs = [width(la_L), width(lb_L), width(lc_L)]
	return legs


if __name__ == '__main__':
	1==1#do nothing
	
