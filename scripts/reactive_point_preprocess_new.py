#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
import sys
import operator
import matplotlib.pyplot as plt
from scipy.spatial import distance
import numpy as np
from visualization_msgs.msg import Marker


x_inter = []
y_inter = []
z_inter = []
x = []
y = []
z = []
ds_thres = None

def interpolation(p1, p2, dis):
	global x, y, z, x_inter, y_inter, z_inter, ds_thres
	num_inter_points = round(dis,3)//ds_thres
	# print dis, num_inter_points
	for i in np.linspace(0,1,num_inter_points + 1):
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
	global ds_thres
	flag = True
	# title_num = int(sys.argv[-1])
	# title = sys.argv[1][title_num:-5]
	rospy.init_node("node2")
	pubRaw = rospy.Publisher("vis_raw", Marker, queue_size=10)
	pub = rospy.Publisher("raw_points", Point, queue_size=10)
	file = open(sys.argv[1], 'r')
	ds_thres = float(sys.argv[2])
	ip_thres = float(sys.argv[3])
	pub_flag = True
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
	x_down = []
	y_down = []
	z_down = []
	times = []	
	count = 0
	count_inter = 0
	rospy.sleep(0.5)
	for i in xrange(len(fl)):
		if "RWrist" in fl[i]:
			start_time = rospy.get_time()
			count += 1
			x_tmp = float(fl[i+9][11:])
			y_tmp = float(fl[i+10][11:])
			z_tmp = float(fl[i+11][11:])
			if len(x) == 0:
				xRaw.append(x_tmp)
				yRaw.append(y_tmp)
				zRaw.append(z_tmp)
			if len(x) >= 1:
				if abs(x[-1] - x_tmp) > 0.1 or abs(y[-1] - y_tmp) > 0.1 or abs(z[-1] - z_tmp) > 0.1:
					continue
				dis = distance.euclidean((x_tmp, y_tmp, z_tmp),(x[-1], y[-1], z[-1]))
			
			if len(x) == 0 or (len(x) >= 1 and dis >= ds_thres):
				xRaw.append(x_tmp)
				yRaw.append(y_tmp)
				zRaw.append(z_tmp)
				# if len(x) >= 1:
				# 	print "Num: ", count
				# 	print dis, round(dis,3)
				if len(x) >= 1 and round(dis,3) >= ip_thres:
					count_inter += 1
					if len(x_temp) != 0:
						# print dis, len(x_temp), x_temp[0]
						# flag = False
						# pass
						for j in xrange(len(x_temp)-1):
							dis_tmp = distance.euclidean((x_temp[j], y_temp[j], z_temp[j]),(x_temp[j+1], y_temp[j+1], z_temp[j+1]))
							if round(dis_tmp,3) >= ip_thres:
								interpolation((x_temp[j], y_temp[j], z_temp[j]),(x_temp[j+1], y_temp[j+1], z_temp[j+1]), dis_tmp)
							else:
								# print "ok", count
								x.append(x_temp[j])
								y.append(y_temp[j])
								z.append(z_temp[j])
								x_down.append(x_temp[j])
								y_down.append(y_temp[j])
								z_down.append(z_temp[j])
								
						dis_tmp = distance.euclidean((x_tmp, y_tmp, z_tmp),(x_temp[-1], y_temp[-1], z_temp[-1]))
						# print x_temp[0], dis_tmp
						if round(dis_tmp,3) >= ip_thres:
							interpolation((x_temp[-1], y_temp[-1], z_temp[-1]), (x_tmp, y_tmp, z_tmp), dis_tmp)
						else:
							# print "ok2", count
							x.append(x_temp[-1])
							y.append(y_temp[-1])
							z.append(z_temp[-1])
							x_down.append(x_temp[-1])
							y_down.append(y_temp[-1])
							z_down.append(z_temp[-1])
							
						x_temp = []
						y_temp = []
						z_temp = []
					else:
						# print dis
						interpolation([x[-1], y[-1], z[-1]], [x_tmp, y_tmp, z_tmp], dis)
				else:
					x_temp = []
					y_temp = []
					z_temp = []
				x.append(x_tmp)
				y.append(y_tmp)
				z.append(z_tmp)
			else:
				# print count
				x_temp.append(x_tmp)
				y_temp.append(y_tmp)
				z_temp.append(z_tmp)
				x_temp_all.append(x_tmp)
				y_temp_all.append(y_tmp)
				z_temp_all.append(z_tmp)
			end_time = rospy.get_time()
			times.append(end_time - start_time)
			# for k in xrange(len(x)):
			# 	point = Point()
			# 	point.x = x[k]
			# 	point.y = y[k]
			# 	point.z = z[k]
			# 	pub.publish(point)
			# 	rospy.sleep(0.047)

	print max(times)
	# # print count_inter
	# fig = plt.figure()
	# fig.set_size_inches(20,10)
	# fig.suptitle("Raw, processed and interpolated points for " + str(title) + "\nDownsampling Threshold: " + str(ds_thres) + "m, Interpolation Threshold: " + str(ip_thres) + "m")
	# ax = plt.axes()
	# ax.scatter(xRaw, yRaw, s=50, label="Raw points")
	# ax.scatter(x_inter, y_inter, s=50, c="red", label="Interpolated points")
	# # ax.scatter(x_temp_all, y_temp_all, s=50, c="green")
	# ax.scatter(x_down, y_down, c="magenta", s=50, label="Points included due to interpolation needs")
	# ax.scatter(x, y, s=20, c="orange", label="Processed points")
	# ax.set_xlabel("x(m)")
	# ax.set_ylabel("y(m)")


	# dis = {}
	# for i in xrange(len(x)-1):
	# 	dis[i+1] = (distance.euclidean((x[i], y[i], z[i]), (x[i+1], y[i+1], z[i+1])))
	# 	# print dis[-1]
	# 	if dis[i+1] < ds_thres:
	# 		# print "ok"
	# 		# print dis[-1]
	# 		pass
	# dis_sort = sorted(dis.items(), key=operator.itemgetter(1))
	# for i in dis_sort:
	# 	print i	
	# ax.legend()
	# ax.grid()
	# plt.savefig("/home/thanasis/Desktop/figs/"+title)
	# plt.show()
	rospy.sleep(0.5)
	if pub_flag:
		i, j = 0, 0
		flag = True
		# print len(x)
		markerRaw = Marker()
		while flag:
			if i < len(xRaw):
				# print i
				point = Point()
				point.x = xRaw[i] + 0.6
				point.y = yRaw[i] + 0.4
				point.z = zRaw[i]

				markerRaw.header.frame_id = "base_link"
				markerRaw.header.stamp = rospy.Time.now()
				markerRaw.action = markerRaw.ADD
				markerRaw.type = markerRaw.LINE_STRIP
				markerRaw.pose.position.x = 0
				markerRaw.pose.position.y = 0
				markerRaw.pose.position.z = 0
				markerRaw.pose.orientation.x = 0
				markerRaw.pose.orientation.y = 0
				markerRaw.pose.orientation.z = 0
				markerRaw.pose.orientation.w = 1
				
				markerRaw.points.append(point)
				markerRaw.scale.x = 0.01
				# markerRaw.scale.y = 0.1
				# markerRaw.scale.z = 0.1
				markerRaw.color.a = 1.0
				markerRaw.color.r = 1.0
				markerRaw.color.g = 0.0
				markerRaw.color.b = 0.04
				markerRaw.lifetime = rospy.Duration(100)
				i += 1
				pubRaw.publish(markerRaw)
			if j < len(x):
				# print j
				pointProc = Point()
				pointProc.x = x[j]
				pointProc.y = y[j]
				pointProc.z = z[j]
				j += 1
				pub.publish(pointProc)
			if i==1:
				rospy.sleep(5)
			else:
				rospy.sleep(0.047)
			if i == len(xRaw) and j == len(x):
				flag = False
		# for i in xrange(len(x)):
		# 	point = Point()
		# 	point.x = x[i]
		# 	point.y = y[i]
		# 	point.z = z[i]
		# 	pub.publish(point)
		# 	if i == 0:
		# 		rospy.sleep(5)
		# 	else:
		# 		rospy.sleep(0.047)
		rospy.loginfo("Published all the points")
		rospy.spin()

main()