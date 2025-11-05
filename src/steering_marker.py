#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Int32
from visualization_msgs.msg import Marker

# Parameters
ANGLE_MIN = -math.radians(90)   # adjust based on your LIDAR!
ANGLE_INCREMENT = math.radians(0.3515625)  # adjust based on your LIDAR!

marker_pub = rospy.Publisher('/disparity_markers', Marker, queue_size=10)
indices = []
distances = []
chosen_idx = -1

def indices_callback(msg):
    global indices
    indices = msg.data

def distances_callback(msg):
    global distances
    distances = msg.data

def chosen_callback(msg):
    global chosen_idx
    chosen_idx = msg.data

def publish_markers(event):
    for i, (idx, dist) in enumerate(zip(indices, distances)):
        angle = ANGLE_MIN + idx * ANGLE_INCREMENT
        x = dist * math.cos(angle)
        y = dist * math.sin(angle)
        m = Marker()
        m.header.frame_id = "car_0_base_link"
        m.header.stamp = rospy.Time.now()
        m.ns = "disparity"
        m.id = 200 + i
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.2
        m.scale.x = 0.13
        m.scale.y = 0.13
        m.scale.z = 0.13
        if i == chosen_idx:
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 1.0
        else:
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 1.0
        marker_pub.publish(m)
    
if __name__ == '__main__':
    rospy.init_node("disparity_marker_visualizer")
    rospy.Subscriber("/disparity_indices", Int32MultiArray, indices_callback)
    rospy.Subscriber("/disparity_distances", Float32MultiArray, distances_callback)
    rospy.Subscriber("/chosen_gap", Int32, chosen_callback)
    rospy.Timer(rospy.Duration(0.2), publish_markers)
    rospy.spin()
