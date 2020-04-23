#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
import sys
import operator
import matplotlib.pyplot as plt
from scipy.spatial import distance
import numpy as np
from visualization_msgs.msg import Marker

x_inter, y_inter, z_inter = [], [], []
x, y, z = [], [], []
x_all, y_all, z_all = [], [], []
x_temp, y_temp, z_temp = [], [], []
xRaw, yRaw, zRaw = [], [], []
x_all, y_all, z_all = [], [], []
x_down, y_down, z_down = [], [], []
ds_thres = None
pub_flag = True
first_point = True
count = 0
count_inter = 0
pub = None

def interpolation(p1, p2, dis):
	global x, y, z, x_inter, y_inter, z_inter, ds_thres, x_all, y_all, z_all
	num_inter_points = round(dis,3)//ds_thres
	for i in np.linspace(0,1,num_inter_points + 1):
		if i==0 or i==1:
			continue
		x_inter.append((1-i)*p1[0] + i*p2[0])
		y_inter.append((1-i)*p1[1] + i*p2[1])
		z_inter.append((1-i)*p1[2] + i*p2[2])
		x.append((1-i)*p1[0] + i*p2[0])
		y.append((1-i)*p1[1] + i*p2[1])
		z.append((1-i)*p1[2] + i*p2[2])
		x_all.append((1-i)*p1[0] + i*p2[0])
		y_all.append((1-i)*p1[1] + i*p2[1])
		z_all.append((1-i)*p1[2] + i*p2[2])


def listened_to_the_points(msg):
	global count, x, y, z, ds_thres, ip_thres, x_all, y_all, z_all, pub, first_point
	if len(x) != 0:
		x = [x[-1]]
		y = [y[-1]]
		z = [z[-1]]

	count += 1
	x_point = msg.x
	y_point = msg.y
	z_point = msg.z

	if len(x) >= 1:
		dis = distance.euclidean((x_point, y_point, z_point),(x[-1], y[-1], z[-1]))
	
	if len(x) == 0 or (len(x) >= 1 and dis >= ds_thres):
		if len(x) >= 1 and dis > ip_thres:
			interpolation((x_point, y_point, z_point),(x[-1], y[-1], z[-1]), dis)
		x.append(x_point)
		y.append(y_point)
		z.append(z_point)
		x_all.append(x_point)
		y_all.append(y_point)
		z_all.append(z_point)	


	if first_point:
		point = Point()
		point.x = x[-1]
		point.y = y[-1]
		point.z = z[-1]
		pub.publish(point)
		rospy.loginfo("Published the first point")
		first_point = False
	else:
		for k in xrange(1, len(x)):
			point = Point()
			point.x = x[k]
			point.y = y[k]
			point.z = z[k]
			pub.publish(point)
			rospy.sleep(0.005)


def main():
	global ds_thres, ip_thres, pub
	rospy.init_node("node3")
	pub = rospy.Publisher("trajectory_points", Point, queue_size=10)
	ds_thres = float(sys.argv[1])
	ip_thres = float(sys.argv[2])
	sub = rospy.Subscriber("raw_points", Point, listened_to_the_points)
	
	rospy.spin()			


if __name__ == "__main__":
	main()