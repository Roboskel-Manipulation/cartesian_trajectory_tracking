#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
import sys
import matplotlib.pyplot as plt
from scipy.spatial import distance
import numpy as np

x_inter = []
y_inter = []
z_inter = []
x = []
y = []
z = []


def interpolation(p1, p2):
	global x, y, z, x_inter, y_inter, z_inter
	for i in np.linspace(0,1,5):
		if i==0 or i==1:
			continue
		# print i
		x_inter.append((1-i)*p1[0] + i*p2[0])
		y_inter.append((1-i)*p1[1] + i*p2[1])
		z_inter.append((1-i)*p1[2] + i*p2[2])
		x.append((1-i)*p1[0] + i*p2[0])
		y.append((1-i)*p1[1] + i*p2[1])
		z.append((1-i)*p1[2] + i*p2[2])


def main():
	rospy.init_node("node2")
	pub = rospy.Publisher("raw_points", Point, queue_size=10)
	file = open(sys.argv[1], 'r')
	ds_thres = float(sys.argv[2])
	ip_thres = float(sys.argv[3])
	pub_flag = False
	fl = file.readlines()
	x_temp = []
	y_temp = []
	z_temp = []
	xRaw = []
	yRaw = []
	zRaw = []
	x_temp_all = []
	y_temp_all = []
	z_temp_all = []
	count = 0
	count_inter = 0
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
				if len(x) >= 1 and dis >= ip_thres:
					count_inter += 1
					# print dis
					if len(x_temp) != 0:
						# pass
						for j in xrange(len(x_temp)-1):
							dis_tmp = distance.euclidean((x_temp[j], y_temp[j], z_temp[j]),(x_temp[j+1], y_temp[j+1], z_temp[j+1]))
							if dis_tmp > ip_thres:
								interpolation((x_temp[j], y_temp[j], z_temp[j]),(x_temp[j+1], y_temp[j+1], z_temp[j+1]))
							else:
								x.append(x_temp[j])
								y.append(y_temp[j])
								z.append(z_temp[j])
						dis_tmp = distance.euclidean((x_tmp, y_tmp, z_tmp),(x_temp[-1], y_temp[-1], z_temp[-1]))
						# print dis_tmp
						if dis_tmp > ip_thres:
							interpolation((x_temp[-1], y_temp[-1], z_temp[-1]), (x_tmp, y_tmp, z_tmp))
						else:
							x.append(x_temp[-1])
							y.append(y_temp[-1])
							z.append(z_temp[-1])
						x_temp = []
						y_temp = []
						z_temp = []
					else:
						print "ok"
						print dis
						interpolation([x[-1], y[-1], z[-1]], [x_tmp, y_tmp, z_tmp])
				else:
					x_temp = []
					y_temp = []
					z_temp = []
				x.append(x_tmp)
				y.append(y_tmp)
				z.append(z_tmp)
			else:
				x_temp.append(x_tmp)
				y_temp.append(y_tmp)
				z_temp.append(z_tmp)
				x_temp_all.append(x_tmp)
				y_temp_all.append(y_tmp)
				z_temp_all.append(z_tmp)
	print count_inter
	fig = plt.figure()
	ax = plt.axes()
	ax.scatter(xRaw, yRaw, s=50)
	ax.scatter(x_inter, y_inter, s=100, c="red")
	# ax.scatter(x_temp_all, y_temp_all, s=100, c="green")
	ax.scatter(x, y, s=20, c="orange")
	for i in xrange(len(x)-1):
		dis=(distance.euclidean((x[i], y[i], z[i]), (x[i+1], y[i+1], z[i+1])))
		if dis >= 0.03:
			# print "ok"
			ax.scatter(x[i], y[i], c="yellow")
	# print max(dis)
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