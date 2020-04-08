#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Point

def main():
	rospy.init_node("points_extraction_from_yaml")
	pub = rospy.Publisher("raw_points", Point, queue_size=100)

	xRaw, yRaw, zRaw = [], [], []
	x, y, z = [], [], []
	file = open(sys.argv[1], 'r')
	fl = file.readlines()
	times = list()
	count = 0
	case = sys.argv[-1]
	for i in xrange(len(fl)):
		if case == '0':
			if "RWrist" in fl[i]:
				point = Point()
				point.x = float(fl[i+9][11:])
				point.y = float(fl[i+10][11:])
				point.z = float(fl[i+11][11:])
				time = int(fl[i+5][16:])
				count += 1
				if time == 0:
					continue
				if len(times) == 0 or (len(times) >= 1 and times[-1] <= time):
					times.append(time)
				else:
					print i
				if count == 1:
					rospy.sleep(2)
					pub.publish(point)
					rospy.sleep(5)
					print "Published first point"
				else:
					rospy.sleep(0.047)
					pub.publish(point)
					print "Published other point"
		elif case == '1':
			if 'x' in fl[i]:
				point = Point()
				point.x = float(fl[i][3:])
				point.y = float(fl[i+1][3:])
				point.z = float(fl[i+2][3:])
				count += 1
				if count == 1:
					rospy.sleep(2)
					pub.publish(point)
					rospy.sleep(5)
					print "Published first point"
				else:
					rospy.sleep(0.047)
					pub.publish(point)
					print "Published other point"
		else:
			if 'x' in fl[i]:
				point = Point()
				point.x = float(fl[i][7:])
				point.y = float(fl[i+1][7:])
				point.z = float(fl[i+2][7:])
				count += 1
				if count == 1:
					rospy.sleep(2)
					pub.publish(point)
					rospy.sleep(5)
					print "Published first point"
				else:
					rospy.sleep(0.047)
					pub.publish(point)
					print "Published other point"

	# point = Point()
	# point.x = 0
	# point.y = 0
	# point.z = 0
	# rospy.sleep(0.047)
	# pub.publish(point)
	print "Published the points"

main()