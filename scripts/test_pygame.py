#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import PointStamped

def main():
	rospy.init_node('yaml_read')
	pub = rospy.Publisher("trajectory_points", PointStamped, queue_size=10)
	file = open(sys.argv[1], 'r')
	fl = file.readlines()
	times = []
	j=0
	for i in range(len(fl)):
		if "RW" in fl[i]:
			keypoint = PointStamped()
			keypoint.header.stamp = rospy.Time(float(fl[i+5][16:-1]+'.'+fl[i+6][17:].replace(' ', '0')))
			keypoint.point.x = float(fl[i+9][11:])
			keypoint.point.y = float(fl[i+10][11:])
			keypoint.point.z = float(fl[i+11][11:])
			try:
				if times[-1] > keypoint.header.stamp.to_sec() + 1 and keypoint.header.stamp.to_sec() != 0:
					rospy.logwarn("End of points")
					break
			except Exception as e:
				print (e)
			pub.publish(keypoint)
			j += 1
			rospy.loginfo("Published keypoint %d" %j)
			rospy.sleep(0.047)
			times.append(keypoint.header.stamp.to_sec())

	rospy.loginfo("Published all keypoints")

main()
