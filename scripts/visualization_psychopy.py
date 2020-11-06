#!/usr/bin/env python3
from psychopy import visual, core
import rospy
from geometry_msgs.msg import PointStamped
from keypoint_3d_matching_msgs.msg import *
from trajectory_execution_msgs.msg import PoseTwist


sizex, sizey = 1000, 1000
radius = 0.01

win = visual.Window([sizex, sizey])
circle = visual.Circle(win, radius=radius, fillCollor='red', lineColor='red')
circle.autoDraw = True

def callback_human(msg):
	global win, circle
	circle.pos = [msg.point.x, msg.point.y]
	win.flip()

def callback_robot(msg):
	global win, circle


def main():
	rospy.init_node('visualization_node')
	
	sub_human = rospy.Subscriber("trajectory_points", PointStamped, callback_human)
	sub_robot = rospy.Subscriber("manos_cartesian_velocity_controller/ee_state", PoseTwist, callback_robot)
	rospy.spin()


main()