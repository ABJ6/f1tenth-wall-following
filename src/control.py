#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float64

# PID Control Params
kp = .8 #TODO
kd = 0.1 #TODO
ki = 0.0 #TODO
angle = 0.0
servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0
integral = 0.0



# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward.
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
vel_input = 1.0	#TODO


# Publisher for moving the car.
# TODO: Use the coorect topic /car_x/offboard/command. The multiplexer listens to this topic
command_pub = rospy.Publisher('/car_0/offboard/command', AckermannDrive, queue_size = 1)
def control(data):
	global prev_error
	global prev_vel
	global vel_input
	global kp
	global kd
	global angle 
	global integral
	frequency = .03

	#print("PID Control Node is Listening to error")
	turningangle = data.pid_error
	min_vel = data.pid_vel

	angle = max(-100, min(angle, 100))

	# 1. Scale the error
	# 2. Apply the PID equation on error to compute steerin
	

	#if errorabs > .5:
	#	speed = max(vel_input - errorabs * 20, 30)
	#if errorabs > .8:
	#	speed = max(vel_input - errorabs * 20, 20)

	#TODO: Tune
	#if errorabs <= 0.3:
	#	speed = vel_input
	#elif errorabs <= 0.8:
	#	normalized = (errorabs - 0.5) / (1.2 - 0.5)
	#	factor = 1 - (normalized ** 2)
	#	speed = (0.7 * vel_input) + factor * (0.3 * vel_input)
	#elif errorabs <= 2.0:
	#	normalized = (errorabs - 1.2) / (2.0 - 1.2)
	#	factor = 1 - (normalized ** 2)
	#	speed = MIN_SPEED + factor * ((0.7 * vel_input) - MIN_SPEED)
	
	#else:
	#	speed = MIN_SPEED
	

	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()

	# TODO: Make sure the steering value is within bounds [-100,100]
	command.steering_angle = math.degrees(turningangle) * 1.5
	# TODO: Make sure the velocity is within bounds [0,100]
	print(min_vel)
	command.speed = min_vel

	# Move the car autonomously
	command_pub.publish(command)

#def find_gap(data, d, n):
#	i = 0
#	seq = []
#	result = []
#	for number in data.ranges:
#		if number > d:
#			seq.append(number)
#		else:
#			if len(seq) > n:
#				result.append(seq)
#				print(i)
#			seq = []
#		i = i+1
#	if len(seq) > n:
#		result.append(seq)
#		print(i)
#	return result



def callback(data):
	global forward_projection

if __name__ == '__main__':

    # This code tempalte asks for the values for the gains from the user upon start, but you are free to set them as ROS parameters as well.")
	vel_input = input("Enter desired velocity: ")
	rospy.init_node('pid_controller', anonymous=True)
    # subscribe to the error topic
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()

