import ik_helper
import numpy as np


def rot_matrix(angle):
	angle_r = angle/180.0*np.pi
	rotation_matrix = np.matrix([ [np.cos(angle_r), -np.sin(angle_r)],
		[np.sin(angle_r), np.cos(angle_r)] ])
	return rotation_matrix



if __name__ == '__main__':
	lead = (0.25/20.0)*25.4 #mm/revolution of the motor output shaft
	gap = 67.47 #mm from CAD
	hinge_hole_to_outer_edge = 8.54 ## mm measured value
	width_max = gap - 2*(hinge_hole_to_outer_edge)


	print lead, width_max, width_max*(2*lead)


