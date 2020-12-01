#!/usr/bin/env python
import rospy
from keypoint_3d_matching_msgs.msg import Keypoint3d_list
from geometry_msgs.msg import PointStamped


def callback(msg):
	for i in range(len(msg.keypoints)):
		if 'RW' in msg.keypoints[i].name:
			point = PointStamped()
			point.point.x = msg.keypoints[i].points.point.x
			point.point.y = msg.keypoints[i].points.point.y
			point.point.z = msg.keypoints[i].points.point.z
			point.header.stamp = msg.keypoints[i].points.header.stamp
			pub.publish(point)
			break

rospy.init_node('check_time')
sub = rospy.Subscriber('/transform_topic', Keypoint3d_list, callback)
pub = rospy.Publisher('trajectory_points', PointStamped, queue_size=100) 
rospy.sleep(0.2)
rospy.spin()