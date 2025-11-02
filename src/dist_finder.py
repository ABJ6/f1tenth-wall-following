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
	newranges = find_disparity(data,0.1,0.2,data.angle_increment)
	masked   = mask_unsafe(newranges, safety_dist=1.5)
	best_index, max_dist = find_widest_gap_center(masked,.5,3,data.angle_increment,math.radians(-90),1)
	#print(best_index)
	#print(max_dist)
	#langle = angle + 30.0
	#mindegree = math.degrees(data.angle_min)
	#maxdegree = math.degrees(data.angle_max)
	#print("ang_min "+ str(mindegree))
	#print("ang_max "+ str(maxdegree))
	#incridegree = math.degrees(data.angle_increment)
	#print("inc "+ str(incridegree))
	#index = int((langle - mindegree) / incridegree)
	#index = max(0, min(index, len(data.ranges) - 1 ))
	#distance = data.ranges[index]	
	#if math.isinf(distance) or math.isnan(distance):
	#	return 0.0
	target_angle = math.radians(-90) + best_index * data.angle_increment
	
	#target_angle = max(-math.radians(90), min(math.radians(90), target_angle))

	# print( "angle " + str(angle)+" distance " + str(distance))
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
    #TODO: implement
	return target_angle
def find_gap(ranges):
	max_dist = 0
	best_index = 0
	for i in range(0,len(ranges)):
		if not math.isnan(ranges[i]) and not math.isinf(ranges[i]):
			if ranges[i] > max_dist:
				max_dist = ranges[i]
				best_index = i
	return best_index, max_dist


def find_disparity(data, disparity_ths, half_width, angle_increment):
	dispartities = []
	ranges, start_idx, end_idx = crop_lidar(data, fov_degrees=180)
	#print(ranges[256])
	#print(ranges)
	#print(len(ranges))
	for i in range(len(ranges) -1 ):
		d1, d2 = ranges[i], ranges[i + 1]

		if math.isinf(d1) or math.isinf(d2):
			continue

		if abs(d1 - d2 ) > disparity_ths:
			index_near = i if d1 < d2 else i + 1
			index_far = i + 1 if d1 < d2 else i 


			closer_distance = ranges[index_near]

			if closer_distance > 0 and closer_distance > half_width:
				angle = 2 * math.asin(half_width / closer_distance)
				num_samples_to_extend = int(angle / angle_increment)
			else:
				num_samples_to_extend = 0
				
			dispartities.append({
				'dist_right': d1,
				'index_right' : i,
				'dist_left': d2,
				'index_left' : i+1,
				'num_sampels_to_extend': num_samples_to_extend
			})
	#print(dispartities)
	for disp in dispartities:
		closer_dist = min(disp['dist_right'], disp['dist_left'])

		if disp['dist_right'] < disp["dist_left"]:
			start = disp['index_left']
			direction = 1
		else:
			start = disp['index_right']
			direction = -1
		for r in range(disp['num_sampels_to_extend']):
			index = start + r *direction
			if 0 <= index < len(ranges):
				if math.isnan(ranges[index]):
					ranges[index] = closer_dist
				ranges[index] = min(ranges[index], closer_dist)
	#print(ranges)
	return ranges
def find_widest_gap_center(ranges, min_safe=0.5, max_valid=3.0,angle_inc = 0,angle_min = 2,turn_penality = 0):
    capped = [min(r, max_valid) if not math.isinf(r) else 0 for r in ranges]
    #print(capped)
    #print(len(capped))
    gaps = []
    in_gap = False
    start = 0

    for i, r in enumerate(capped):
        if r > min_safe:
            if not in_gap:
                start = i
                in_gap = True
        else:
            if in_gap:
                gaps.append((start, i - 1))
                in_gap = False
    if in_gap:
        gaps.append((start, len(capped) - 1))

    if not gaps:
        return None, None

    best_gaps=None
    best_score = None

    for (s, e) in gaps:
        mid = (s + e) //2
        mid_dist = capped[mid]
        mid_angle = angle_min + mid * angle_inc
        score = mid_dist - turn_penality * abs(mid_angle)
        if score > best_score:
             best_score = score
             best_gap = (mid, mid_dist)
    if best_gap is None:
        return None,None
    return best_gap


def crop_lidar(data, fov_degrees=180):
	half_fov = math.radians(fov_degrees/2)
	angle_min = data.angle_min
	angle_inc = data.angle_increment 
	start_angle = -half_fov
	end_angle = half_fov
	start_idx = int((start_angle - angle_min)/ angle_inc)
	end_idx = int((end_angle -angle_min)/ angle_inc)
	start_idx = max(0, start_idx)
	end_idx = min(len(data.ranges) - 1, end_idx)
	return list(data.ranges[start_idx:end_idx]), start_idx, end_idx

def mask_unsafe(ranges, safety_dist=1.5):
    out = []
    for r in ranges:
        if r is None or math.isnan(r) or math.isinf(r):
            out.append(0.0)        # invalid = blocked
        elif r < safety_dist:
            out.append(0.0)        # too close = blocked
        else:
            out.append(r)          # far enough = open
    return out


def callback(data):
	global forward_projection
	

	theta = 65 # you need to try different values for theta
	angle = getRange(data, theta-120) # obtain the ray distance for theta
	
	#b = getRange(data, -120)	# obtain the ray distance for 0 degrees (i.e. directly to the right of the car)nt()
	##print("b "+ str(b))
	#swing = math.radians(theta)
	#if a is None or b is None or abs(math.sin(swing)) < 1e-6: 
	#	error = 0
	#	return

	

	## Your code goes here to determine the projected error as per the alrorithm
	# Compute Alpha, AB, and CD..and finally the error.
	#try:
	#	Alpha = math.atan((a * math.cos(swing) - b) / (a * math.sin(swing)))
	#except ZeroDivisionError:
	#	error = 0
	#	return
	#AB = b* math.cos(Alpha)

	#CD = AB + forward_projection * math.sin(Alpha)
#	print("a " + str(a))
	#print("b "+ str(b))
	#print("AB" + str(AB))
	#print("Alpha" + str(Alpha))
	#error = desired_distance - CD
	#print("CD" + str(CD))
	#print("error"+ str(error))
	

	#if 2.69 <= error <= 2.7:
	#	error = 0

	msg = pid_input()	# An empty msg is created of the type pid_input
	# this is the error that you want to send to the PID for steering correction.
	msg.pid_error = angle
	print(angle)
	#msg.pid_vel = vel		# velocity error can also be sent.
	pub.publish(msg)
	#print(data.ranges)
	#print(len(data.ranges))


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_0/scan",LaserScan,callback)
	rospy.spin()
