#!/usr/bin/env python
import rospy
from trajectory_execution_msgs.msg import PointArray
from keypoint_3d_matching_msgs.msg import Keypoint3d_list
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from trajectory_process_utils_srvs.srv import *

import numpy as np
from scipy.spatial import distance

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
pub_all = None
sum_time = 0
second_point = False
init_x, init_y, init_z = None, None, None
start_bz_time = None
init_point = True

def callback(data):
	global init_point, start_bz_time, init_x, init_y, init_z, second_point, sum_time, count, pub, pub_all, xV_tmp, yV_tmp, zV_tmp, x, y, z, xFinal, yFinal, zFinal, xRaw, yRaw, zRaw, xMov, yMov, zMov, start_threshold, start_flag, end_flag
	# rospy.loginfo("Received point")
	# x_tmp = data.point.x
	# y_tmp = data.point.y
	# z_tmp = data.point.z

	for i in range(len(data.keypoints)):
		if (data.keypoints[i].name == "RWrist"):
			print ('received point')
			x_tmp = data.keypoints[i].points.point.x
			y_tmp = data.keypoints[i].points.point.y
			z_tmp = data.keypoints[i].points.point.z
			timestamp = data.keypoints[i].points.header.stamp.to_sec()
			break

	if end_flag:
		if x_tmp != 0 and y_tmp != 0 and z_tmp != 0:
			if len(xRaw) == 0 or (len(xRaw) >= 1 and abs(xRaw[-1] - x_tmp) < 0.1 and abs(yRaw[-1] - y_tmp) < 0.1 and abs(zRaw[-1] - z_tmp) < 0.1):
				xRaw.append(x_tmp)
				yRaw.append(y_tmp)
				zRaw.append(z_tmp)
	# print (x_tmp, y_tmp, z_tmp)
	count += 1
	if init_point and x_tmp < 0 and x_tmp >= -0.45 and y_tmp < 0 and y_tmp >= -0.45:
		point = PointStamped()
		point.point.x = x_tmp
		point.point.y = y_tmp
		point.point.z = z_tmp
		init_x = x_tmp
		init_y = y_tmp
		init_z = z_tmp
		pub.publish(point)
		rospy.loginfo("Num of control points: %d"%count)
		second_point = True
		init_point = False

	if len(x) == 1 and count != 1:
		# start_bz_time = timestamp
		pass
		
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
							start_flag = True
						if start_flag:
							xMov.append(x_tmp)
							yMov.append(y_tmp)
							zMov.append(z_tmp)
							x.append(x_tmp)
							y.append(y_tmp)
							z.append(z_tmp)
							# print (len(x))
							if len(x) == 4:
								# for i in range(len(data.keypoints)):
								# 	if (data.keypoints[i].name == "RWrist"):
								# 		end_bz_time = data.keypoints[i].points.header.stamp.to_sec()
								# 		break
								try:
									rospy.wait_for_service("trajectory_smoothing")
									smoothing = rospy.ServiceProxy("trajectory_smoothing", Smoothing)
									resp = smoothing(x, y, z)
									x = resp.x_smooth
									y = resp.y_smooth
									z = resp.z_smooth
									x_all = resp.x_smooth_all
									y_all = resp.y_smooth_all
									z_all = resp.z_smooth_all
									
									rospy.loginfo("Smoothed the trajectory")
									xFinal.extend(x)
									yFinal.extend(y)
									zFinal.extend(z)
									# rospy.loginfo(start_bz_time)
									# rospy.loginfo(end_bz_time)
									# bz_duration = end_bz_time - start_bz_time
									
									# rospy.loginfo("%f"%bz_duration)
									if len(x) > 1:
										# pub_rate = bz_duration/(len(x)-1)
										pub_rate = (3*0.047)/(len(x)-1)
									for i in xrange(1, len(x)):
										point = PointStamped()
										point.point.x = x[i]
										point.point.y = y[i]
										point.point.z = z[i]
										if i==1:
											point.point.z = z[i] + 10
										else:
											point.point.z = z[i]
										if second_point:
											second_point = False
											dis = distance.euclidean([x[i], y[i], z[i]], [init_x, init_y, init_z])
											num_points = dis//0.012 - 1
											for j in np.linspace(0, 1, num_points):
												if j!= 0:
													point = PointStamped()
													point.point.x = (1-j)*init_x + j*x[i]
													point.point.y = (1-j)*init_y + j*y[i]
													point.point.z = (1-j)*init_z + j*z[i]
													pub.publish(point)
													rospy.sleep(0.001)
										else:
											pub.publish(point)
											rospy.sleep(pub_rate)
										# pub.publish(point)
										# rospy.sleep(pub_rate)
									
									# for i in xrange(len(x_all)):
									# 	point = Point()
									# 	point.x = x_all[i]
									# 	point.y = y_all[i]
									# 	point.z = z_all[i]
									# 	pub_all.publish(point)
									# 	rospy.sleep(0.0005)									
									
									end_time = rospy.get_time()
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
								end_flag = False
					
def movement_detection_node():
	rospy.init_node("movement_detection_node")
	global pub, pub_all, start_threshold
	rospy.loginfo("Ready to record NEW movement")
	start_threshold = 24
	pub = rospy.Publisher("trajectory_points", PointStamped, queue_size=10)	
	pub_all = rospy.Publisher("trajectory_points_all", Point, queue_size=10)	
	# sub = rospy.Subscriber("raw_points", Point, callback)
	# sub = rospy.Subscriber("raw_points_online", Keypoint3d_list, callback)
	sub = rospy.Subscriber("raw_points", Keypoint3d_list, callback)
	rospy.spin()


if __name__ == '__main__':
	movement_detection_node()

