#def ik_pos_cb(msg):
	# if msg.buttons[8]:
	# 	command3.updateZ(axes_map(msg.axes[3],z_max,z_min))
	# 	command3.updateCommand()
	# if msg.buttons[0]:
	# 	command3.updateXY(msg.axes[0],msg.axes[1],msg.axes[3])
	# 	command3.updateCommand()

	# if msg.buttons[9]:
	# 	command2.updateZ(axes_map(msg.axes[3],z_max,z_min))
	# 	command2.updateCommand()
	# if msg.buttons[1]:
	# 	command2.updateXY(msg.axes[0],msg.axes[1],msg.axes[3])
	# 	command2.updateCommand()

	# if msg.buttons[11]:
	# 	command1.updateZ(axes_map(msg.axes[3],z_max,z_min))
	# 	command1.updateCommand()
	# if msg.buttons[2]:
	# 	command1.updateXY(msg.axes[0],msg.axes[1],msg.axes[3])
	# 	command1.updateCommand()

	# print command
	# command[0:3] = command1.getCommand()
	# print command
	# command[3:6] = command2.getCommand()
	# print command
	# command[6:] = command3.getCommand()
	# print command

	#Potentially include if statement for which stage is moving, current setup can have all stages move at the same time. But technically only one stage's numbers can be
	#changed at a time

# def ik_cb(msg):
# 	global z

# 	max_radius = 4

# 	x = msg.axes[0]
# 	y = msg.axes[1]
# 	r = np.sqrt(x**2+y**2)


# 	#increment z
# 	if msg.buttons[8]:
# 		z = axes_map(msg.axes[3],z_max,z_min)


# 	if msg.buttons[0] and r>0:
# 		drad = axes_map(msg.axes[3],max_radius,0)
# 		x_corrected = drad*(x/r)
# 		y_corrected = drad*(y/r)
# 		x =x_corrected
# 		y = y_corrected
# 	else:
# 		x = 0
# 		y = 0


# 	# #constrain x,y setpoint to some max radius
# 	# if r>=max_radius:
# 	# 	x_corrected = max_radius*(x/r)
# 	# 	y_corrected = max_radius*(y/r)
# 	# 	x =x_corrected
# 	# 	y = y_corrected


# 	width_sps = ik_legs(x,y,z)


# 	stage = 3



# 	stage_index1 = 3*(stage-1)
# 	stage_index2 = stage_index1+3


# 	if np.any(np.isnan(width_sps)):
# 		print "nan", x,y, z
# 	else:
# 		command[ stage_index1 : stage_index2 ] = width_sps
# 		command_msg = Float32MultiArray(data = command)
# 		ik_pub.publish(command_msg)
