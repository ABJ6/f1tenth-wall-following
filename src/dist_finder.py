#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

from std_msgs.msg import Int32MultiArray, Float32MultiArray, Int32
from geometry_msgs.msg import PolygonStamped
from visualization_msgs.msg import Marker

disparity_indices_pub = rospy.Publisher('/disparity_indices', Int32MultiArray, queue_size=10)
disparity_distances_pub = rospy.Publisher('/disparity_distances', Float32MultiArray, queue_size=10)
chosen_gap_pub = rospy.Publisher('/chosen_gap', Int32, queue_size=10)
footprint_pub = rospy.Publisher('/footprint', PolygonStamped, queue_size=10)
pub = rospy.Publisher('error', pid_input, queue_size =10)
# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 1.2	# distance (in m) that we project the car forward for correcting the error. You have to adjust this.
desired_distance = 1	# distance from the wall (in m). (defaults to right wall). You need to change this for the track
#vel = 15 		# this vel variable is not really used here.
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.
prev_best = 0
# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
disparity_indices_pub = rospy.Publisher('/disparity_indices', Int32MultiArray, queue_size=10)
disparity_distances_pub = rospy.Publisher('/disparity_distances', Float32MultiArray, queue_size=10)
chosen_gap_pub = rospy.Publisher('/chosen_gap', Int32, queue_size=10)
chosen_distance = rospy.Publisher('/chosen_distance',Int32, queue_size=10 )


# PID values = 55, 5, 0

def getRange(data,angle):
	newranges, min_distance, disparities = find_disparity(data,0.3,0.25,data.angle_increment)
	masked   = mask_unsafe(newranges, safety_dist=1.5)
	best_index, max_dist = find_widest_gap_center(masked,.5,3,data.angle_increment,math.radians(-90),1.0)
	if best_index is not None:
		target_angle = math.radians(-90) + best_index * data.angle_increment
	#except Exception as e:
	#	target_angle = math.radians(-90) + 256 * data.angle_increment

	
	#target_angle = max(-math.radians(90), min(math.radians(90), target_angle))

	# print( "angle " + str(angle)+" distance " + str(distance))
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
    #TODO: implement
	return target_angle, min_distance, best_index, disparities, max_dist
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
	min_distance = 3
	for disp in dispartities:
		closer_dist = min(disp['dist_right'], disp['dist_left'])
		if closer_dist < min_distance:
			min_distance = closer_dist
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
	return ranges, min_distance, dispartities

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

    best_index = None
    max_dist = -1

    for (s, e) in gaps:
        for i in range(s, e + 1):
            dist = capped[i] - turn_penality * abs(angle_min + i * angle_inc)
            if dist > max_dist:
                max_dist = dist
                best_index = i
    if best_index is None:
        return 0, 0
    #print(best_index)
    return best_index, capped[best_index]

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


def DVS(data, turningangle):
	# Parameters to tune
	v_max = 25     # Max speed
	v_min = 10     # Min speed
	k_angle = 120  # Higher sensitivity = slower for sharper turns, tune as needed
	k_obs = 40     # Higher = more drop as obstacles get closer

    # Assumes your main node passes latest laser scan readings as a global, or you fetch them here
	if data is not None:
        # Use the minimum distance as obstacle metric, set a cap for very far obstacles
		min_dist = min([r for r in data.ranges if not math.isnan(r) and not math.isinf(r)], 3.5)
		min_dist = min(min_dist, 3)  # Use 3.5 meters as open
	else:
		min_dist = 1.0  # fallback

    # 1. Obstacle-based velocity: faster if objects are farther away
	v_obs = v_min + (v_max - v_min) * (min_dist / 3.5)  # Linear scaling

    # 2. Turn-based velocity: slower for sharper steering
	steering = abs(math.degrees(turningangle))
	v_turn = v_max - k_angle * (steering / 90.0)  # Scaling, 90 deg = sharpest
	v_turn = max(v_min, min(v_turn, v_max))

    # 3. Select the lower of the two for safety
	target_vel = min(v_obs, v_turn)
	if 'prev_vel' in globals():
		alpha = 0.2
		target_vel = alpha * target_vel + (1-alpha) * prev_vel  # smooth!
	prev_vel = target_vel



	# 1. Scale the error
	# 2. Apply the PID equation on error to compute steering
	target_vel = max(v_min, min(target_vel, v_max))

	return target_vel


def callback(data):
	global forward_projection
	

	theta = 65 # you need to try different values for theta
	angle, min_distance, best_index,dispartities, chosen_distance_return = getRange(data, theta-120)
	#speed = DVS(data,angle) # obtain the ray distance for theta
	
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
	#msg.pid_vel = speed
	#print(angle)
	#msg.pid_vel = gap_distance
	#msg.pid_vel = vel		# velocity error can also be sent.
	pub.publish(msg)
	indices = []
	distances = []
	for disp in dispartities:
		indices.append(disp['index_right'])
		distances.append(min(disp['dist_right'], disp['dist_left']))
	chosen = best_index if best_index is not None else -1
	disparity_indices_pub.publish(Int32MultiArray(data=indices))
	disparity_distances_pub.publish(Float32MultiArray(data=distances))
	chosen_gap_pub.publish(Int32(data=chosen))
	chosen_distance.publish(Int32(data=chosen_distance_return))
	#print(data.ranges)
	#print(len(data.ranges))


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_0/scan",LaserScan,callback)
	rospy.spin()
