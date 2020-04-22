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
	fl = file.readlines()
	title_num = int(sys.argv[-1])
	title_day = sys.argv[-2]
	title = sys.argv[1][title_num:-5] + "_" + sys.argv[-2]
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
	count_down = 0
	down_list = []
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
				down_list.append(count_down)
				count_down = 0
				if len(x) >= 1 and dis > ip_thres:
					# print dis
					num_points = round(dis,3)//ds_thres
					print dis, num_points
					for j in np.linspace(0,1,num_points + 1):
						if j==0 or j==1:
							continue
						x_inter.append((1-j)*x[-1] + j*x_tmp)
						y_inter.append((1-j)*y[-1] + j*y_tmp)
						z_inter.append((1-j)*z[-1] + j*z_tmp)
						x.append((1-j)*x[-1] + j*x_tmp)
						y.append((1-j)*y[-1] + j*y_tmp)
						z.append((1-j)*z[-1] + j*z_tmp)
				x.append(x_tmp)
				y.append(y_tmp)
				z.append(z_tmp)
			else:
				count_down += 1
	down_list.append(count_down)

	fig, ax = plt.subplots(1,2)
	fig.set_size_inches(20,10)
	ax[0].set_title("Raw, processed and downsampled points for " + str(title) + "\nDownsampling Threshold: " + str(ds_thres) + "m, Interpolation Threshold: " + str(ip_thres) + "m")
	# fig.suptitle("Smart interpolation")
	ax[0].scatter(xRaw, yRaw, s=100, label="Raw points")

	dis = []
	for i in xrange(len(x)-1):
		dis_tmp = distance.euclidean((x[i], y[i], z[i]), (x[i+1], y[i+1], z[i+1]))
		dis.append(dis_tmp)
		if dis_tmp <= 0.012:
			ax[0].text(x[i], y[i], str(1))
	# ax.scatter(x_inter, y_inter, s=100, c="red", label="Interpolated points")
	# ax[0].scatter(x_temp_all, y_temp_all, s=50, c="red", label="Downsampled points")
	# ax.scatter(x_down, y_down, c="magenta", s=50, label="Points included due to interpolation needs")
	ax[0].scatter(x, y, s=20, c="orange", label="Processed points")
	ax[0].set_xlabel("x(m)")
	ax[0].set_ylabel("y(m)")
	ax[0].grid()
	ax[0].legend()

	ax[1].set_title("Bar plot of waitlists")
	langs = list(set(down_list))
	freq = []
	for i in set(down_list):
		freq.append(down_list.count(i))
	ax[1].bar(langs, freq)
	plt.xticks(list(set(down_list)))
	plt.yticks(list(set(freq)))
	ax[1].set_xlabel("Number of downsampled points per two consecutive points")
	ax[1].set_ylabel("Frequency")
	ax[1].grid()

	file = open("./distances.txt", 'a')
	# plt.savefig("/home/thanasis/Desktop/reactive_yaml_files/figs/" + title)
	
	file.write(title + " %f, %f\n"%(min(dis), max(dis)))
	file.close()
	

	# print min(dis), max()	

	plt.show()


main()