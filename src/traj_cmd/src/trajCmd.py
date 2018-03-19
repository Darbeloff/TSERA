#!/usr/bin/python


import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy


traj_pub = rospy.Publisher('/traj',Float32MultiArray,queue_size = 1)


def traj_cb(msg):

	if(msg.buttons[11]==1):
		print "Going home"
		sp_cmd = Float32MultiArray(data = [1, 0.001, 0.001, 60 ] ) #stage , x, y, z
		traj_pub.publish(sp_cmd)
		rospy.sleep(5)
		sp_cmd = Float32MultiArray(data = [2, 0.001, 0.001, 60 ] ) #stage , x, y, z
		traj_pub.publish(sp_cmd)
		rospy.sleep(5)
		sp_cmd = Float32MultiArray(data = [3, 0.001, 0.001, 60 ] ) #stage , x, y, z
		traj_pub.publish(sp_cmd)
		rospy.sleep(5)
		print "Arrived Home"

	elif (msg.buttons[10] == 1):
		print "Trajectory begins"
		for i in range(20,100+1):
			if (i<40):
				j = 1
			elif i<70:
				j = 2
			else:
				j = 3


			sp_cmd = Float32MultiArray(data = [j, 0.001, 0.001, i ] ) #stage , x, y, z
			traj_pub.publish(sp_cmd)
			rospy.sleep(0.1)
		print "Trajectory Completed"

def traj_cmd():
	print 'Trajectory command initialized...'
	rospy.init_node('trajCmd')
	rospy.Subscriber('/joy', Joy, traj_cb )


if __name__ == '__main__':
	traj_cmd()
	rospy.spin()