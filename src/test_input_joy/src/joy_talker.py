#!/usr/bin/python
import rospy
from std_msgs.msg import Float32MultiArray, String
pub = rospy.Publisher('/des_ort', Float32MultiArray, queue_size = 1)

def cb(msg):
	vector = msg.data.split()
	vector_list = [0,0,1,0,0,1,float(vector[0]),float(vector[1]),float(vector[2]), 8]
	vector_msg = Float32MultiArray(data = vector_list)
	pub.publish(vector_msg)

def talker():
	rospy.init_node('grad_calc')
	rospy.Subscriber("/test", String, cb)
	rospy.spin()

if __name__ == '__main__':
	talker()