#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
import sys
import matplotlib.pyplot as plt
from scipy.spatial import distance
import numpy as np

def main():
	rospy.init_node("node")
	pub = rospy.Publisher("raw_points", Point, queue_size=10)
	file = open(sys.argv[1], 'r')
	ds_thres = float(sys.argv[2])
	ip_thres = float(sys.argv[3])
	pub_flag = False
	fl = file.readlines()
	x = list()
	y = list()
	z = list()
	xRaw = list()
	yRaw = list()
	zRaw = list()
	x_inter = list()
	y_inter = list()
	z_inter = list()
	count = 0
	for i in xrange(len(fl)):
		if "RWrist" in fl[i]:
			count += 1
			x_tmp = float(fl[i+9][11:])
			y_tmp = float(fl[i+10][11:])
			z_tmp = float(fl[i+11][11:])
			xRaw.append(x_tmp)
			yRaw.append(y_tmp)
			zRaw.append(z_tmp)
			if len(x) >= 1:
				if abs(x[-1] - x_tmp) > 0.1 or abs(y[-1] - y_tmp) > 0.1 or abs(z[-1] - z_tmp) > 0.1:
					continue
				dis = distance.euclidean((x_tmp, y_tmp, z_tmp),(x[-1], y[-1], z[-1]))
			if len(x) == 0 or (len(x) >= 1 and dis >= ds_thres):
				if len(x) >= 1 and dis > ip_thres:
					print dis
					for j in np.linspace(0,1,5):
						if j==0 or j==1:
							continue
						# print j
						x_inter.append((1-j)*x[-1] + j*x_tmp)
						y_inter.append((1-j)*y[-1] + j*y_tmp)
						z_inter.append((1-j)*z[-1] + j*z_tmp)
						x.append((1-j)*x[-1] + j*x_tmp)
						y.append((1-j)*y[-1] + j*y_tmp)
						z.append((1-j)*z[-1] + j*z_tmp)
				x.append(x_tmp)
				y.append(y_tmp)
				z.append(z_tmp)

	fig = plt.figure()
	ax = plt.axes()
	ax.scatter(xRaw, yRaw, s=50)
	ax.scatter(x_inter, y_inter, s=100, c="red")
	ax.scatter(x, y, s=20, c="orange")
	ax.grid()
	plt.show()

	if pub_flag:
		for i in xrange(len(x)):
			point = Point()
			point.x = x[i]
			point.y = y[i]
			point.z = z[i]
			pub.publish(point)
			if i == 0:
				rospy.sleep(5)
			else:
				rospy.sleep(0.047)
		rospy.loginfo("Published all the points")
		rospy.spin()

main()