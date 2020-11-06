#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from keypoint_3d_matching_msgs.msg import *
from visualization_msgs.msg import Marker
from scipy.spatial import distance


def main():
	rospy.init_node('yaml_read')
	pub = rospy.Publisher("raw_points_online", Keypoint3d_list, queue_size=10)
	file = open(sys.argv[1], 'r')
	fl = file.readlines()
	times = []
	rospy.sleep(2)
	
	pubRaw = rospy.Publisher("vis_raw", Marker, queue_size=100)
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
	j=0
	for i in range(len(fl)):
		point = Keypoint3d_list()
		if "RW" in fl[i]:
			keypoint = Keypoint3d()
			keypoint.name = "RWrist"
			keypoint.points.header.stamp = rospy.Time(float(fl[i+5][16:-1]+'.'+fl[i+6][17:].replace(' ', '0')))
			keypoint.points.point.x = float(fl[i+9][11:])
			keypoint.points.point.y = float(fl[i+10][11:])
			keypoint.points.point.z = float(fl[i+11][11:])
			try:
				if times[-1] > keypoint.points.header.stamp.to_sec() + 1 and keypoint.points.header.stamp.to_sec() != 0:
					rospy.logwarn("End of points")
					break
			except Exception as e:
				print (e)
			point.keypoints.append(keypoint)
			point_marker = Point()
			point_marker.x = keypoint.points.point.x
			point_marker.y = keypoint.points.point.y
			point_marker.z = keypoint.points.point.z
			markerRaw.points.append(point_marker)
			pubRaw.publish(markerRaw)
			pub.publish(point)
			j += 1
			rospy.loginfo("Published keypoint %d" %j)
			rospy.sleep(0.047)
			# try:
			# 	rospy.sleep(keypoint.points.header.stamp.to_sec()-times[-1])
			# 	rospy.loginfo('Slept for ' + str(keypoint.points.header.stamp.to_sec() - times[-1]))
			# except Exception as ex:
			# 	rospy.logwarn(ex)
			# 	rospy.sleep(10)
			times.append(keypoint.points.header.stamp.to_sec())

	rospy.loginfo("Published all keypoints")

main()
