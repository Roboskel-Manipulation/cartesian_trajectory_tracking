#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
from keypoint_3d_matching_msgs.msg import *
from visualization_msgs.msg import Marker

import sys
import numpy as np
import random
import matplotlib.pyplot as plt
import time
import collections

def main():
	rospy.init_node("points_extraction_from_yaml")
	pub = rospy.Publisher("trajectory_points", PointStamped, queue_size=100)
	pubRaw = rospy.Publisher("vis_raw", Marker, queue_size=100)
	init_point = True
	x, y, z = [], [], []
	times = []
	
	markerRaw = Marker()
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
	markerRaw.scale.x = 0.01
	markerRaw.color.a = 1.0
	markerRaw.color.r = 1.0
	markerRaw.color.g = 0.0
	markerRaw.color.b = 0.0
	markerRaw.lifetime = rospy.Duration(100)
	start = False
	samples = 0
	count = 0
	start = False

	# x = list(np.linspace(0.25, 0.45, 50))
	# x.extend(np.linspace(0.45, 0.25, 50))
	# x.extend(np.linspace(0.25, 0.45, 50))
	# x.extend(np.linspace(0.45, 0.25, 50))
	# x.extend(np.linspace(0.25, 0.45, 50))
	# x.extend(np.linspace(0.45, 0.25, 50))

	# y = [0.116 for i in range(len(x))]
	# z = [0.05 for i in range(len(x))]
	# rospy.sleep(0.5)
	# for i in range(len(x)):
	# 	point = PointStamped()
	# 	# if i==0:
	# 	# 	point.header.stamp = rospy.Time.now()
	# 	# else:
	# 	# 	point.header.stamp = rospy.Time(timestamp.to_sec() + sleep_rate)
	# 	point.header.stamp = rospy.Time.now()
	# 	timestamp = point.header.stamp
	# 	point.point.x = x[i]
	# 	point.point.y = y[i]
	# 	point.point.z = z[i]
	# 	pub.publish(point)
	# 	sleep_rate = (random.random()+0.02)*0.08/1.02
	# 	print (sleep_rate)
	# 	# rospy.sleep(10) if i == 0 else rospy.sleep(sleep_rate)
	# 	rospy.sleep(10) if i == 0 else rospy.sleep(0.047)
	# rospy.loginfo("Published all the points")
		
	file = open(sys.argv[1], 'r')
	fl = file.readlines()
	points = Keypoint3d_list()
	sleep_times = []
	points, times_now, times_yaml = [], [], []
	sleep_times_now = []
	
	for i in xrange(len(fl)):
		if "RW" in fl[i]:
			keypoint = Keypoint3d()
			keypoint.name = "RWrist"
			keypoint.points.point.x = float(fl[i+9][11:])
			keypoint.points.point.y = float(fl[i+10][11:])
			keypoint.points.point.z = float(fl[i+11][11:])
			time_point = float(fl[i+5][16:-1]+'.'+fl[i+6][17:].replace(' ', '0'))
			time_point_now = rospy.Time.now().to_sec()


			if times_now is not None:
				try:
					# sleep_rate = time_point - times[-1]
					# sleep_rate = float(time_point_now) - float(times_now[-1])
					# sleep_rate = times_now[-1] - times_now[-2]
					# sleep_rate = np.float64(rospy.Time(0.002).to_sec()) - np.float64(rospy.Time(0.001).to_sec()) #*(rospy.Time.now().to_sec() - times_now[-1])
					# sleep_rate = np.float64(rospy.Time.now().to_sec()) - np.float64(rospy.Time.now().to_sec()-1) #*(rospy.Time.now().to_sec() - times_now[-1])
					sleep_rate = np.float64(time_point_now - time_last)/2 #/(rospy.Time.now().to_sec() - times_now[-1])
					x = sleep_rate + 1
					rospy.sleep(sleep_rate)
					# sleep_times_now.append(sleep_rate)
					rospy.loginfo("Slept for %f secs"%sleep_rate)
					# print (time_point_now, times_now[-1])
				except Exception as err:
					rospy.logwarn(err)

				
			time_last = time_point_now
			# points.append(keypoint.points.point.x)
			# times.append(time_point)
			# times_now.append(np.float64(time_point_now))
			# time2 = rospy.Time.now().to_sec()

	times_dur_now = []
	for i in range(len(times_now)-1):
		times_dur_now.append(times_now[i+1]-times_now[i])

	times_dur = []
	for i in range(len(times)-1):
		times_dur.append(times[i+1]-times[i])

	fig = plt.figure()
	ax = plt.axes()
	ax.scatter(np.linspace(0, len(sleep_times_now), len(sleep_times_now)), sleep_times_now)


	fig, ax = plt.subplots(1,2)
	ax[0].set_title('Time delay now')
	ax[0].scatter(np.linspace(0, len(times_dur_now), len(times_dur_now)), times_dur_now, s=20)
	ax[0].set_xlabel('Samples(N)')
	ax[0].set_ylabel('times(s)')
	ax[0].grid()

	ax[1].set_title('Time delay yaml')
	ax[1].scatter(np.linspace(0, len(times_dur), len(times_dur)), times_dur, s=20)
	ax[1].set_xlabel('Samples(N)')
	ax[1].set_ylabel('times(s)')
	ax[1].grid()

	# for i in xrange(len(fl)):
	# 	if "RWrist" in fl[i]:
	# 		keypoint = Keypoint3d()
	# 		keypoint.name = "RWrist"
	# 		keypoint.points.point.x = float(fl[i+9][11:])
	# 		keypoint.points.point.y = float(fl[i+10][11:])
	# 		keypoint.points.point.z = float(fl[i+11][11:])
	# 		time_point = float(fl[i+5][16:-1] + '.' + fl[i+6][17:].replace(' ', '0'))
			
	# 		try:
	# 			rospy.sleep(time_point - times[-1])
	# 			sleep_times_now.append(rospy.Time.now().to_sec() - times_now[-1])
	# 			sleep_times.append(time_point - times[-1])
	# 			rospy.loginfo("Slept for %f secs"%(time_point-times[-1]))
	# 		except Exception as e:
	# 			rospy.logwarn(e)
	# 		times.append(time_point)

	# 		if len(x) == 0 or (abs(x[-1]-keypoint.points.point.x) < 0.1 and abs(y[-1]-keypoint.points.point.y) < 0.1 and abs(z[-1]-keypoint.points.point.z) < 0.1):
	# 			x.append(keypoint.points.point.x)
	# 			y.append(keypoint.points.point.y)
	# 			z.append(keypoint.points.point.z)
	# 		else:
	# 			continue

	# 		if init_point:
	# 			count += 1
	# 			init_point = False
	# 			rospy.sleep(0.5)
	# 			rospy.loginfo('Waiting 10 secs so that the end effector can reach the starting point')
	# 			point = PointStamped()
	# 			point.header.stamp = rospy.Time.now()
	# 			point.point.x = keypoint.points.point.x
	# 			point.point.y = keypoint.points.point.y
	# 			point.point.z = keypoint.points.point.z
	# 			pub.publish(point)
	# 			points.append(point.point.x)
	# 			times_now.append(point.header.stamp.to_sec())
	# 			times_yaml.append(time_point)
	# 			# rospy.sleep(10)
	# 		else:
	# 			if len(x) >= 2:
	# 				if len(x) == 25:
	# 					x.pop(0)
	# 					y.pop(0)
	# 					z.pop(0)
	# 				std_x = np.std(x)
	# 				std_y = np.std(y)
	# 				std_z = np.std(z)
	# 				if not start and (std_x > 0.01 or std_y > 0.01 or std_z > 0.01):
	# 					start = True
	# 				if start:
	# 					count += 1
	# 					point = PointStamped()
	# 					point.header.stamp = rospy.Time.now()
	# 					point.point.x = keypoint.points.point.x
	# 					point.point.y = keypoint.points.point.y
	# 					point.point.z = keypoint.points.point.z
	# 					pub.publish(point)
	# 					points.append(point.point.x)
	# 					times_now.append(point.header.stamp.to_sec())
	# 					times_yaml.append(time_point)
	# 					# rospy.loginfo('Published %dth point and gonna wait for %f secs'%(count, sleep_rate))
	# 					if std_x <= 0.01 and std_y <= 0.01:
	# 						rospy.loginfo('Motion Ended')
	# 						break
	
	# rospy.loginfo('Mean value of sleep rates: %f'%np.mean(sleep_times))
	# rospy.loginfo('Total ellapsed time: %f'%(times[-1]-times[0]))

	# del times_now[0]
	# del times[0]
	# del points[0]
	times_now = [i-times_now[0] for i in times_now]
	times = [i-times[0] for i in times]

	# Plot x(t)
	fig = plt.figure()
	ax = plt.axes()
	ax.scatter(times, points, s=20, label='Openpose points')
	ax.scatter(times_now, points, s=20, label='Openpose points - NOW')
	ax.set_xlabel('time(s)')
	ax.set_ylabel('x(m)')
	ax.grid()
	ax.legend()

	# fig = plt.figure()
	# ax = plt.axes()
	# ax.scatter(np.linspace(0, len(sleep_times), len(sleep_times)), sleep_times, label='Sleep times')
	# ax.scatter(np.linspace(0, len(sleep_times_now), len(sleep_times_now)), sleep_times_now, label='Sleep times - NOW')
	# ax.set_xlabel('time(s)')
	# ax.set_ylabel('x(m)')
	# ax.grid()
	# ax.legend()
	plt.show()

main()

