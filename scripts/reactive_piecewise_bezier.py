#!/usr/bin/env python
import rospy
from trajectory_execution_msgs.msg import PointArray
from keypoint_3d_matching_msgs.msg import Keypoint3d_list
from geometry_msgs.msg import Point
from trajectory_process_utils_srvs.srv import *

import numpy as np


xV_tmp, yV_tmp, zV_tmp = [], [], []
x, y, z = [], [], []
xFinal, yFinal, zFinal = [], [], []
xRaw, yRaw, zRaw = [], [], []
xMov, yMov, zMov = [], [], []
count = 0
start_flag = False
end_flag = False
start_threshold = None
pub = None
sum_time = 0

def callback(data):
	global sum_time, count, pub, xV_tmp, yV_tmp, zV_tmp, x, y, z, xFinal, yFinal, zFinal, xRaw, yRaw, zRaw, xMov, yMov, zMov, start_threshold, start_flag, end_flag
	start_time = rospy.get_time()
	x_tmp = data.x
	y_tmp = data.y
	z_tmp = data.z
	if end_flag:
		if x_tmp != 0 and y_tmp != 0 and z_tmp != 0:
			if len(xRaw) == 0 or (len(xRaw) >= 1 and abs(xRaw[-1] - x_tmp) < 0.1 and abs(yRaw[-1] - y_tmp) < 0.1 and abs(zRaw[-1] - z_tmp) < 0.1):
				xRaw.append(x_tmp)
				yRaw.append(y_tmp)
				zRaw.append(z_tmp)
	count += 1
	if count == 1:
		point = Point()
		point.x = x_tmp
		point.y = y_tmp
		point.z = z_tmp
		# rospy.sleep(0.5)
		pub.publish(point)
		rospy.loginfo("Num of control points: %d"%count)
	if not end_flag:
		if x_tmp != 0 and y_tmp != 0 and z_tmp != 0:
			if len(xRaw) == 0 or (len(xRaw) >= 1 and abs(xRaw[-1] - x_tmp) < 0.1 and abs(yRaw[-1] - y_tmp) < 0.1 and abs(zRaw[-1] - z_tmp) < 0.1):
				xRaw.append(x_tmp)
				yRaw.append(y_tmp)
				zRaw.append(z_tmp)				
				if abs(x_tmp) < 0.6 and abs(y_tmp) < 0.6 and abs(z_tmp) < 0.6:
					if len(xV_tmp) == start_threshold:
						del xV_tmp[0]
						del yV_tmp[0]
						del zV_tmp[0]
					xV_tmp.append(x_tmp)
					yV_tmp.append(y_tmp)
					zV_tmp.append(z_tmp)
					if len(xV_tmp) >= 2:
						std_x = np.std(xV_tmp)
						std_y = np.std(yV_tmp)
						std_z = np.std(zV_tmp)
						if (not start_flag) and (std_x > 0.01 or std_y > 0.01 or std_z > 0.01):
							print("Start movement at sample %d" %count)
							# for k in xrange(len(xV_tmp)-start_threshold, len(xV_tmp)-1):
							# 	x.append(xV_tmp[k])
							# 	y.append(yV_tmp[k])
							# 	z.append(zV_tmp[k])					
							start_flag = True
						if start_flag:
							xMov.append(x_tmp)
							yMov.append(y_tmp)
							zMov.append(z_tmp)
							x.append(x_tmp)
							y.append(y_tmp)
							z.append(z_tmp)
							if len(x) == 4:
								try:
									rospy.wait_for_service("trajectory_smoothing")
									smoothing = rospy.ServiceProxy("trajectory_smoothing", Smoothing)
									resp = smoothing(x, y, z)
									x = resp.x_smooth
									y = resp.y_smooth
									z = resp.z_smooth
									rospy.loginfo("Smoothed the trajectory")
									xFinal.extend(x)
									yFinal.extend(y)
									zFinal.extend(z)
									print (len(x))
									for i in xrange(len(x)):
										point = Point()
										point.x = x[i]
										point.y = y[i]
										if i == 1:
											point.z = z[i] + 10
										else:
											point.z = z[i]
										pub.publish(point)
										rospy.sleep(0.0000005)
									end_time = rospy.get_time()
									sum_time += end_time - start_time
									x = [x[-1]]
									y = [y[-1]]
									z = [z[-1]]
									# break
									# if count-56 > 5:
									# 	break
								except rospy.ServiceException, e:
									rospy.logerr("Service call failed: %s"%e)	

							if std_x <= 0.01 and std_y <= 0.01 and std_z <= 0.01:
								print("End movement at sample %d" %count)
								rospy.loginfo("Time elapsed: %f" %sum_time)
								end_flag = True
					
def movement_detection_node():
	rospy.init_node("movement_detection_node")
	global pub, start_threshold
	rospy.loginfo("Ready to record NEW movement")
	start_threshold = 24
	pub = rospy.Publisher("trajectory_points", Point, queue_size=10)	
	sub = rospy.Subscriber("raw_points", Point, callback)
	rospy.spin()


if __name__ == '__main__':
	movement_detection_node()
