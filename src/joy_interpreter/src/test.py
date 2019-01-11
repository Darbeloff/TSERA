import ik_helper
import numpy as np


def rot_matrix(angle):
	angle_r = angle/180.0*np.pi
	rotation_matrix = np.matrix([ [np.cos(angle_r), -np.sin(angle_r)],
		[np.sin(angle_r), np.cos(angle_r)] ])
	return rotation_matrix



if __name__ == '__main__':


	print ik_helper.ik_legs(0,0,92)


