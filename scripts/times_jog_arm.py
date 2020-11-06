#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import numpy as np

v = []

def callback (msg):
	global v
	v.append(msg.data)
	if len(v) >= 100:
		rospy.loginfo("Mean: %f"%np.mean(v))
		rospy.loginfo("Std: %f"%np.std(v))

def main():
	rospy.init_node('times')
	sub = rospy.Subscriber('times_topic', Float64, callback)
	rospy.spin()

main()