#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import PointStamped
from keypoint_3d_matching_msgs.msg import *
from visualization_msgs.msg import Marker

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
			if len(x) == 0 or (abs(x[-1]-keypoint.points.point.x) < 0.1 and abs(y[-1]-keypoint.points.point.y) < 0.1 and abs(z[-1]-keypoint.points.point.z) < 0.1):
				x.append(keypoint.points.point.x)
				y.append(keypoint.points.point.y)
				z.append(keypoint.points.point.z)
			else:
				continue
			try:
				rospy.sleep(time_point - times[-1])
			except Exception as e:
				rospy.loginfo(e)

			times.append(time_point)
			if init_point:
				init_point = False
				rospy.sleep(0.5)
				point = PointStamped()
				point.header.stamp = rospy.Time(time_point)
				point.point.x = keypoint.points.point.x
				point.point.y = keypoint.points.point.y
				point.point.z = keypoint.points.point.z
				pub.publish(point)
				rospy.sleep(10)
			else:
				point = PointStamped()
				point.header.stamp = rospy.Time(time_point)
				point.point.x = keypoint.points.point.x
				point.point.y = keypoint.points.point.y
				point.point.z = keypoint.points.point.z
				pub.publish(point)
				sleep_rate = times[-1]-times[-2]
main()