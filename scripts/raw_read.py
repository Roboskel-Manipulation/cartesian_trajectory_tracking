#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from keypoint_3d_matching_msgs.msg import *
from visualization_msgs.msg import Marker
from scipy.spatial import distance

def main():
	rospy.init_node("points_extraction_from_yaml")
	pub = rospy.Publisher("raw_points", Keypoint3d_list, queue_size=100)
	pubRaw = rospy.Publisher("vis_raw", Marker, queue_size=100)
	xRaw, yRaw, zRaw = [], [], []
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
	first_point = True

	file = open(sys.argv[1], 'r')
	fl = file.readlines()
	for i in xrange(len(fl)):
		if "RWrist" in fl[i]:
			points = Keypoint3d_list()
			keypoint = Keypoint3d()
			keypoint.name = "RWrist"
			keypoint.points.point.x = float(fl[i+9][11:])
			keypoint.points.point.y = float(fl[i+10][11:])
			keypoint.points.point.z = float(fl[i+11][11:])
			xRaw.append(keypoint.points.point.x)
			yRaw.append(keypoint.points.point.y)
			zRaw.append(keypoint.points.point.z)
			time = int(fl[i+5][16:])
			try:
				print (float(fl[i+5][16:-1] + '.' + fl[i+6][17:].replace(' ', '0')) - time_point)
				# rospy.sleep(float(fl[i+5][16:-1] + '.' + fl[i+6][17:].replace(' ', '0')) - time_point)
				rospy.sleep(0.047)
			except NameError:
				print ("try next time...")
			time_point = float(fl[i+5][16:-1] + '.' + fl[i+6][17:].replace(' ', '0'))

			if len(x) == 0:
				x.append(keypoint.points.point.x)
				y.append(keypoint.points.point.y)
				z.append(keypoint.points.point.z)
				continue
			if len(x) >= 1 and (abs(x[-1] - keypoint.points.point.x) < 0.1 and abs(y[-1] - keypoint.points.point.y) < 0.1 and abs(z[-1] - keypoint.points.point.z) < 0.1):
				x.append(keypoint.points.point.x)
				y.append(keypoint.points.point.y)
				z.append(keypoint.points.point.z)
				keypoint.points.header.stamp = rospy.Time.from_sec(time_point)
				points.keypoints.append(keypoint)
				pub.publish(points)
				if first_point:
					rospy.sleep(0.2)
					rospy.loginfo("Published first point")
					rospy.loginfo("Waiting 5 secs")
					rospy.sleep(5)
					start_time = rospy.get_time()
					first_point = False
				else:
					# pub.publish(point)
					point_marker = Point()
					point_marker.x = keypoint.points.point.x + 0.6
					point_marker.y = keypoint.points.point.y + 0.4
					point_marker.z = keypoint.points.point.z
					# keypoint.points.point.x += 0.6
					# keypoint.points.point.y += 0.4
					markerRaw.points.append(point_marker)
					pubRaw.publish(markerRaw)
					rospy.loginfo("Published other point")
					end_time = rospy.get_time()

	print end_time - start_time
	print "Published the points"

main()