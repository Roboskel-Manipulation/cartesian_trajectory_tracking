#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from keypoint_3d_matching_msgs.msg import *
from visualization_msgs.msg import Marker
from scipy.spatial import distance
import numpy as np

def main():
	rospy.init_node("points_extraction_from_yaml")
	pub = rospy.Publisher("trajectory_points", Point, queue_size=100)
	pubRaw = rospy.Publisher("vis_raw", Marker, queue_size=100)
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

	file = open(sys.argv[1], 'r')
	fl = file.readlines()
	points = Keypoint3d_list()
	for i in xrange(len(fl)):
		if "RWrist" in fl[i]:
			keypoint = Keypoint3d()
			keypoint.name = "RWrist"
			keypoint.points.point.x = float(fl[i+9][11:])
			keypoint.points.point.y = float(fl[i+10][11:])
			keypoint.points.point.z = float(fl[i+11][11:])
			time_point = float(fl[i+5][16:-1] + '.' + fl[i+6][17:].replace(' ', '0'))
			try:
				if (time_point - time_point_prev < 0):
					rospy.loginfo("Human motion ended. Gonna exit...")
					break
			except Exception as e:
				rospy.loginfo(e)
			samples += 1
			if len(x) == 0:
				x.append(keypoint.points.point.x)
				y.append(keypoint.points.point.y)
				z.append(keypoint.points.point.z)
				init_point_x = keypoint.points.point.x
				init_point_y = keypoint.points.point.y
				init_point_z = keypoint.points.point.z
				point = Point()
				point.x = init_point_x
				point.y = init_point_y
				point.z = init_point_z
				rospy.sleep(1)
				pub.publish(point)
				rospy.sleep(10)
				continue
				
			if len(x) >= 1 and (abs(x[-1] - keypoint.points.point.x) < 0.1 and abs(y[-1] - keypoint.points.point.y) < 0.1 and abs(z[-1] - keypoint.points.point.z) < 0.1):
				x.append(keypoint.points.point.x)
				y.append(keypoint.points.point.y)
				z.append(keypoint.points.point.z)
				if (len(x) == 24):
					x.pop(0)
					y.pop(0)
					z.pop(0)
				if len(x) >= 2:
					std_x = np.std(x)
					std_y = np.std(y)
					std_z = np.std(z)
					if (not start) and (std_x >= 0.01 or std_y >= 0.01 or std_z >= 0.01):
						rospy.loginfo("Motion started at sample %d"%samples)
						start = True
						dis_x = keypoint.points.point.x - init_point_x
						dis_y = keypoint.points.point.y - init_point_y
						dis_z = keypoint.points.point.z - init_point_z
					if start:
						point = Point()
						point.x = keypoint.points.point.x
						point.y = keypoint.points.point.y
						point.z = keypoint.points.point.z
						pub.publish(point)
						try:
							rospy.loginfo("Time duration: %f"%(time_point-time_point_prev))
							pub_rate = time_point - time_point_prev
							rospy.sleep(pub_rate)
						except Exception as e:
							rospy.loginfo(e)
							rospy.sleep(0.047)
						time_point_prev = time_point
						point_marker = Point()
						point_marker.x = keypoint.points.point.x + 0.6 - dis_x
						point_marker.y = keypoint.points.point.y + 0.4 - dis_y
						point_marker.z = keypoint.points.point.z + 0.0 - dis_z
						markerRaw.points.append(point_marker)
						pubRaw.publish(markerRaw)
						rospy.loginfo("Published other point")
						if std_x < 0.01 and std_y < 0.01 and std_z < 0.01:
							rospy.loginfo("Motion ended at sample %d"%samples)
							break
	print "Published the points"

main()