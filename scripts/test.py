#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point
from trajectory_execution_msgs.msg import PoseTwist

des = None
pub = None
D = 1000000000000000000000000000000000000000000000000000000

def callback(msg):
	global des, pub, D
	ee_pos = Point()
	ee_pos.x = msg.pose.position.x
	ee_pos.y = msg.pose.position.y
	ee_pos.z = msg.pose.position.z

	vel = Twist()
	vel.linear.x = D*(0.2 - ee_pos.x) 
	vel.linear.y = D*(0.3 - ee_pos.y)
	vel.linear.z = D*(0.1 - ee_pos.z) 
	# rospy.loginfo("x_des: %f"%des.x)
	# rospy.loginfo("y_des: %f"%des.y)
	# rospy.loginfo("z_des: %f"%des.z)
	pub.publish(vel)

def main():
	rospy.init_node('test')
	global des, pub
	# des = Point()
	# des.x = rospy.get_param('/x')
	# des.y = rospy.get_param('/y')
	# des.z = rospy.get_param('/z')

	pub = rospy.Publisher('/manos_cartesian_velocity_controller_sim/command_cart_vel', Twist, queue_size=10)
	sub = rospy.Subscriber('/manos_cartesian_velocity_controller_sim/ee_state', PoseTwist, callback)
	rospy.spin()




main()