#!/usr/bin/env python
import rospy
from trajectory_execution_msgs.msg import PoseTwist
from geometry_msgs.msg import Point, Twist

vel = None
vel_pub = None
desired_point = None
received_point = False

def input_callback(msg):
	global received_point, desired_point
	received_point = True
	desired_point.x = msg.x
	desired_point.y = msg.y
	desired_point.z = msg.z

def state_callback(msg):
	global vel, desired_point, received_point, vel_pub
	if received_point:
		vel.linear.x = desired_point.x - msg.pose.position.x
		vel.linear.y = desired_point.y - msg.pose.position.y
		vel.linear.z = desired_point.z - msg.pose.position.z
		vel_pub.publish(vel)

def main():
	rospy.init_node("test_controller")
	global vel, desired_point, vel_pub
	desired_point = Point()
	vel = Twist()
	vel_pub = rospy.Publisher("manos_cartesian_velocity_controller_sim/command_cart_vel", Twist, queue_size=10)
	state_sub = rospy.Subscriber("manos_cartesian_velocity_controller_sim/ee_state", PoseTwist, state_callback)
	input_sub = rospy.Subscriber("input_points", Point, input_callback)
	rospy.spin()


main()