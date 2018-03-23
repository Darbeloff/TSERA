#!/usr/bin/python

import numpy as np


def la( x, y, z, Lt, s0, r):
	t1a = -Lt/np.sqrt(3) + r + s0 - 2*x
	t2a = np.sqrt(2/3.0*(r-x)*(np.sqrt(3)*Lt + 3*x) - 2*y**2 ) - z
	la = np.sqrt( t1a**2 + t2a**2)
	return la

def lb(x, y, z, Lt, s0, r, M, k):

	
	t1b = r + s0 + x - (Lt+3*y)/np.sqrt(3)
	t2b = 1.0/(6.0*y) * (-3*M*r - 3*M*x + np.sqrt(6)*k*y + 6*y*z)
	lb = np.sqrt( t1b**2 + t2b**2 ) 
	return lb


def lc(x, y, z, Lt, s0, r, M, k):
	
	t1c = r + s0 + x - (Lt-3*y)/np.sqrt(3)
	t2c = 1.0/(6.0*y) * (3*M*r + 3*M*x + np.sqrt(6)*k*y + 6*y*z)
	lc = np.sqrt( t1c**2 + t2c**2 ) 
	return lc


rot_min = 0;
rot_max = 27.47/2*0.7874 # garbage value

def constrain_leg_width(rotations):
	if rotations >= rot_max:
		return rot_max
	elif rotations<=rot_min:
		return rot_min
	else:
		return rotations




phi = 99.59/180*np.pi  #the angle of the corner between the scissor standoff and the imaginary length le 
l0 = 30
sciss_length_0 = 30 + 42.5  + 85 + 42.5
sciss_length_1 = 42.5  + 85 + 42.5 # sum of all scissor lengths (minus offset)
lb = 4 # length of offset

def width(L):

	lp = np.sqrt( l0**2 + lb**2 - 2*l0*lb* np.cos(phi) )
	le = np.sqrt( sciss_length_0**2 + lb**2 - 2* sciss_length_0*lb*np.cos(phi) )
	beta = np.arccos( (sciss_length_1**2 - lp**2 - le**2)/( -2*lp*le ) )

	alpha = np.arcsin(L/le)
	w = 2 * lp * np.cos(alpha + beta)

	#convert to rotations of the motor shaft
	w = (50 -  w)*.7874/2
	


	return constrain_leg_width(w)




def ik_legs(x,y,z):
	'''
	takes x,y,z coordinate in segment base frame and 
	calculates required leg lengths

	'''
	Lt = 57.57 #mm fill in new values with scherical joint
	s0 = 33.23 #mm

	r = np.sqrt(x**2 + y**2)



	la_L = la( x,y, z, Lt, s0, r)

	#Avoid singularity
	if(x!=0.0 and y!=0.0):
		k = np.sqrt(-3*y**2 + (np.sqrt(3)*Lt + 3*x)*(-x + r))
		M = np.sqrt(2*(r - x)*(np.sqrt(3)*Lt + 3*x) - 6*y**2)

		lb_L = lb(x, y, z, Lt, s0, r, M, k)
		lc_L = lc(x, y, z, Lt, s0, r, M, k)
	else:
		lb_L = la_L
		lc_L = la_L


	legs = [width(la_L), width(lb_L), width(lc_L)]
	return legs


if __name__ == '__main__':
	1==1#do nothing
	
