#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
import sys
import matplotlib.pyplot as plt
from scipy.spatial import distance
import numpy as np

def main():
	rospy.init_node("node")
	pub = rospy.Publisher("raw_topic", Point, queue_size=10)
	file = open(sys.argv[1], 'r')
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
			if len(x) == 0 or (len(x) >= 1 and dis >= 0.015):
				if len(x) >= 1 and dis > 0.03:
					for i in np.linspace(0,1,5):
						if i==0 or i==1:
							continue
						print i
						x_inter.append((1-i)*x[-1] + i*x_tmp)
						y_inter.append((1-i)*y[-1] + i*y_tmp)
						z_inter.append((1-i)*z[-1] + i*z_tmp)
						x.append((1-i)*x[-1] + i*x_tmp)
						y.append((1-i)*y[-1] + i*y_tmp)
						z.append((1-i)*z[-1] + i*z_tmp)
				x.append(x_tmp)
				y.append(y_tmp)
				z.append(z_tmp)

	fig = plt.figure()
	ax = plt.axes()
	ax.scatter(xRaw, yRaw, s=50)
	ax.scatter(x_inter, y_inter, s=100, c="red")
	ax.scatter(x, y, s=20, c="orange")
	plt.show()

	for i in xrange(len(x)):
		point = Point()
		point.x = x[i]
		point.y = y[i]
		point.z = z[i]
		pub.publish(point)
	rospy.loginfo("Published all the points")
	rospy.spin()

main()