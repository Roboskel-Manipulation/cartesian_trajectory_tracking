#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
from keypoint_3d_matching_msgs.msg import *
from trajectory_execution_msgs.msg import PoseTwist

import pygame
import threading

xpos_r, ypos_r = 500, 500
xpos_b, ypos_b = 500, 500
radius = 10

pygame.init()
screen = pygame.display.set_mode((1000, 1000))
pygame.display.flip()
sc=1000
last_msg = None
count1, count2 = 0, 0
key_lock = threading.Lock()
initx_human, inity_human, initx_robot, inity_robot = 0, 0, 0, 0

def callback_human(msg):
	global screen, sc, xpos_r, ypos_r, xpos_b, ypos_b, key_lock, count1, initx_human, inity_human
	if count1 == 0:
		initx_human = int(round(sc*(msg.point.x+0.5)))
		inity_human = int(round(sc*(-msg.point.y+0.5)))
	count1 += 1
	xpos_b = int(round(sc*(msg.point.x+0.5))) - initx_human + 500
	ypos_b = int(round(sc*(-msg.point.y+0.5))) - inity_human + 500
	# screen.fill((0, 0, 0))
	# pygame.draw.circle(screen, (255, 0, 0), (xpos_r, ypos_r), radius)
	# pygame.draw.circle(screen, (0, 0, 255), (xpos_b, ypos_b), radius)
	# pygame.display.flip()


def callback_robot(msg):
	global screen, sc, xpos_r, ypos_r, xpos_b, ypos_b, key_lock, count2, initx_robot, inity_robot
	if count2 == 0:
		initx_robot = int(round(sc*(msg.pose.position.x+0.5)))
		inity_robot = int(round(sc*(-msg.pose.position.y+0.5)))
	count2 += 1
	xpos_r = int(round(sc*(msg.pose.position.x+0.5))) - initx_robot + 500
	ypos_r = int(round(sc*(-msg.pose.position.y+0.5))) - inity_robot + 500
	# xpos_r = int(round(sc*(msg.point.x+0.5)))
	# ypos_r = int(round(sc*(-msg.point.y+0.5)))

	# screen.fill((0, 0, 0))
	pygame.draw.circle(screen, (0, 0, 255), (xpos_b, ypos_b), radius)
	pygame.draw.circle(screen, (255, 0, 0), (xpos_r, ypos_r), radius)
	pygame.display.flip()

def main():
	rospy.init_node('visualization_node')
	
	sub_human = rospy.Subscriber("trajectory_points_stamp", PointStamped, callback_human)
	sub_robot = rospy.Subscriber("manos_cartesian_velocity_controller/ee_state", PoseTwist, callback_robot)
	rospy.spin()


main()