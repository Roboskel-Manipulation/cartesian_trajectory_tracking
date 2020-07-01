#!/usr/bin/env python
import rospy
from keypoint_3d_matching_msgs.msg import *

import sys

def main():
	rospy.init_node('camera_points_read')
	pub = rospy.Publisher('camera_keypoint_3d_matching', Keypoint3d_list, queue_size=10)
	rospy.sleep(0.5)
	times = []
	file = open(sys.argv[1], 'r')
	fl = file.readlines()
	for i in range(len(fl)):
		if 'RW' in fl[i]:
			keypoints = Keypoint3d_list()
			point = Keypoint3d()
			point.name = 'RWrist'
			point.points.point.x = float(fl[i+9][11:])
			point.points.point.y = float(fl[i+10][11:])
			point.points.point.z = float(fl[i+11][11:])
			point.points.header.stamp = rospy.Time(float(fl[i+5][16:-1]+'.'+fl[i+6][17:].replace(' ', '0')))
			point.points.header.frame_id = 'camera_rgb_optical_frame'
			try:
				sleep_rate = point.points.header.stamp - times[-1]
				rospy.sleep(sleep_rate)
			except Exception as e:
				rospy.loginfo(e)
			times.append(point.points.header.stamp)
			keypoints.keypoints.append(point)
			pub.publish(keypoints)
	rospy.loginfo('Published all points')


if __name__ == '__main__':
	main()
	