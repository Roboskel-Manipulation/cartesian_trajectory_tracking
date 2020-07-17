#!/usr/bin/env python
import rospy
from trajectory_execution_msgs.msg import PoseTwist
from geometry_msgs.msg import Pose, Twist

count = 0
vel_pub, timestamp = None, None
pose = Pose()

def state_callback(msg):
	global timestamp, count, vel_pub, pose
	if count == 0:
		pose = Pose()
		pose.position.x = msg.pose.position.x
		pose.position.y = msg.pose.position.y
		pose.position.z = msg.pose.position.z
		timestamp = msg.header.stamp
	else:
		try:
			derived_vel = Twist()
			derived_vel.linear.x = (msg.pose.position.x - pose.position.x)/(msg.header.stamp.to_sec() - timestamp.to_sec())
			derived_vel.linear.y = (msg.pose.position.y - pose.position.y)/(msg.header.stamp.to_sec() - timestamp.to_sec())
			derived_vel.linear.z = (msg.pose.position.z - pose.position.z)/(msg.header.stamp.to_sec() - timestamp.to_sec())
			vel_pub.publish(derived_vel)
			rospy.loginfo("Published derived velocity")
			pose = Pose()
			pose.position.x = msg.pose.position.x
			pose.position.y = msg.pose.position.y
			pose.position.z = msg.pose.position.z
			timestamp = msg.header.stamp
		except Exception as err:
			rospy.logwarn(err)
	count += 1

def main():
	global vel_pub
	rospy.init_node("debug_vel")
	sub = rospy.Subscriber("/manos_cartesian_velocity_controller_sim/ee_state", PoseTwist, state_callback)
	pub = rospy.Publisher("/manos_cartesian_velocity_controller_sim/command_cart_vel", Twist, queue_size=10)
	vel_pub = rospy.Publisher("/derived_velocity", Twist, queue_size=10)
	rospy.sleep(0.5)
	com_twist = Twist()
	com_twist.linear.x = -0.1
	pub.publish(com_twist)
	rospy.sleep(2)
	com_twist.linear.x = 0.0
	pub.publish(com_twist)

	rospy.spin()

main()