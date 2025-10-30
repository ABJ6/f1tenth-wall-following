#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 1.2	# distance (in m) that we project the car forward for correcting the error. You have to adjust this.
desired_distance = 1	# distance from the wall (in m). (defaults to right wall). You need to change this for the track
#vel = 15 		# this vel variable is not really used here.
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)

# PID values = 55, 5, 0

def getRange(data,angle):
	langle = angle + 30.0
	mindegree = math.degrees(data.angle_min)
	incridegree = math.degrees(data.angle_increment)
	index = int((langle - mindegree) / incridegree)
	index = max(0, min(index, len(data.ranges) - 1 ))
	distance = data.ranges[index]
	if math.isinf(distance) or math.isnan(distance):
		return 0.0
	
	# print( "angle " + str(angle)+" distance " + str(distance))
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
    #TODO: implement
	return distance



def callback(data):
	global forward_projection
	

	theta = 65 # you need to try different values for theta
	a = getRange(data, theta-120) # obtain the ray distance for theta
	b = getRange(data, -120)	# obtain the ray distance for 0 degrees (i.e. directly to the right of the car)nt()
	##print("b "+ str(b))
	swing = math.radians(theta)
	if a is None or b is None or abs(math.sin(swing)) < 1e-6: 
		error = 0
		return

	

	## Your code goes here to determine the projected error as per the alrorithm
	# Compute Alpha, AB, and CD..and finally the error.
	try:
		Alpha = math.atan((a * math.cos(swing) - b) / (a * math.sin(swing)))
	except ZeroDivisionError:
		error = 0
		return
	AB = b* math.cos(Alpha)

	CD = AB + forward_projection * math.sin(Alpha)
#	print("a " + str(a))
	#print("b "+ str(b))
	#print("AB" + str(AB))
	#print("Alpha" + str(Alpha))
	error = desired_distance - CD
	#print("CD" + str(CD))
	print("error"+ str(error))
	

	if 2.69 <= error <= 2.7:
		error = 0

	msg = pid_input()	# An empty msg is created of the type pid_input
	# this is the error that you want to send to the PID for steering correction.
	msg.pid_error = error
	#msg.pid_vel = vel		# velocity error can also be sent.
	pub.publish(msg)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_0/scan",LaserScan,callback)
	rospy.spin()
