#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from scipy.spatial import distance
import numpy as np

def main():
	rospy.init_node("points_extraction_from_yaml")
	pub = rospy.Publisher("trajectory_points", Point, queue_size=100)
	pubRaw = rospy.Publisher("vis_raw", Marker, queue_size=100)
	x, y, z = [], [], []
	file = open(sys.argv[1], 'r')
	fl = file.readlines()
	times = list()
	
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
	for i in xrange(len(fl)):
		if "RWrist" in fl[i]:
			point = Point()
			point.x = float(fl[i+9][11:])
			point.y = float(fl[i+10][11:])
			point.z = float(fl[i+11][11:])
			time_point = float(fl[i+5][16:-1] + '.' + fl[i+6][17:].replace(' ', '0'))
			samples += 1
			if len(x) == 0:
				x.append(point.x)
				y.append(point.y)
				z.append(point.z)
				init_point_x = point.x
				init_point_y = point.y
				init_point_z = point.z
				rospy.sleep(0.2)
				pub.publish(point)
				rospy.sleep(5)
				continue
			if len(x) >= 1 and (abs(x[-1] - point.x) < 0.1 and abs(y[-1] - point.y) < 0.1 and abs(z[-1] - point.z) < 0.1):
				x.append(point.x)
				y.append(point.y)
				z.append(point.z)
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
						dis_x = point.x - init_point_x
						dis_y = point.y - init_point_y
						dis_z = point.z - init_point_z
					if start:
						pub.publish(point)
						rospy.sleep(0.047)
						point_marker = Point()
						point_marker.x = point.x + 0.6 - dis_x
						point_marker.y = point.y + 0.4 - dis_y
						point_marker.z = point.z + 0.0 - dis_z
						markerRaw.points.append(point_marker)
						pubRaw.publish(markerRaw)
						rospy.loginfo("Published other point")
						if std_x < 0.01 and std_y < 0.01 and std_z < 0.01:
							rospy.loginfo("Motion ended at sample %d"%samples)
							break
	print "Published the points"

main()