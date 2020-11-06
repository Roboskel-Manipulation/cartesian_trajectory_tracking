#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped

def main():
	rospy.init_node('one')
	pub = rospy.Publisher("trajectory_points", PointStamped, queue_size=10)

	point = PointStamped()
	translation = 0.005
	sleep_rate = 0.047
	while not rospy.is_shutdown():
		point.point.y += translation
		pub.publish(point)
		rospy.sleep(sleep_rate)

	rospy.spin()


main()