#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped

def main():
	rospy.init_node('two')
	pub = rospy.Publisher("/manos_cartesian_velocity_controller/ee_state", PointStamped, queue_size=10)

	point = PointStamped()
	translation = 0.005*0.008/0.047
	sleep_rate = 0.008
	while not rospy.is_shutdown():
		point.point.y += translation
		pub.publish(point)
		rospy.sleep(sleep_rate)

	rospy.spin()


main()