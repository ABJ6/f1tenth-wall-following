#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Int32
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler


# Parameters
ANGLE_MIN = -math.radians(90)   
ANGLE_INCREMENT = 0.0061359 

marker_pub = rospy.Publisher('/disparity_markers', Marker, queue_size=10)
cd_pub = rospy.Publisher('/tmarker', Marker, queue_size=10)
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

def chosen_distance(msg):
    global chosen_dis
    chosen_dis = msg.data


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
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0
        marker_pub.publish(m)

    
    if chosen_idx < len(distances):
        angle = ANGLE_MIN + chosen_idx * ANGLE_INCREMENT
        target = Marker()
        target.header.frame_id = "car_0_base_link"
        target.header.stamp = rospy.Time.now()
        target.ns = "target_point"
        target.id = 1
        target.type = Marker.SPHERE
        target.action = Marker.ADD
        range_val = distances[chosen_idx]
        actual_target_scan = 120 + chosen_idx
        angle = ANGLE_MIN + actual_target_scan + ANGLE_INCREMENT
        
        target.pose.position.x = range_val * math.cos(angle)
        target.pose.position.y = range_val * math.sin(angle)
        target.pose.position.z = 0.0
        target.pose.position.orientation.w = 1.0
        target.scale.x = 0.3
        target.scale.y = 0.3
        target.scale.z = 0.3
        target.color.r = 0.0
        target.color.g = 1.0
        target.color.b = 0.0
        target.color.a = 1.0
        marker_pub.publish(target)

    

def steering_callback(cmd):
    """Draw an orange arrow showing current steering."""
    m = Marker()
    m.header.frame_id = "car_0_base_link"
    m.header.stamp    = rospy.Time.now()
    m.ns   = "steering"
    m.id   = 1
    m.type = Marker.ARROW
    m.action = Marker.ADD

    m.pose.position.x = 0.5
    m.pose.position.y = 0.0
    m.pose.position.z = 0.2

    yaw_deg = getattr(cmd, "steering_angle", 0.0)
    yaw_rad = math.radians(yaw_deg)
    q = quaternion_from_euler(0.0, 0.0, yaw_rad)
    m.pose.orientation = Quaternion(*q)

    # Arrow size and color
    m.scale.x = 0.6   # shaft length
    m.scale.y = 0.05  # shaft diameter
    m.scale.z = 0.08  # head diameter
    m.color.r = 1.0
    m.color.g = 0.4
    m.color.b = 0.0
    m.color.a = 1.0

    # Optional: short lifetime so it refreshes smoothly
    m.lifetime = rospy.Duration(0.25)

    # Reuse the same topic you're already using so RViz only needs ONE Marker display
    marker_pub.publish(m)






if __name__ == '__main__':
    rospy.init_node("disparity_marker_visualizer")
    rospy.Subscriber("/disparity_indices", Int32MultiArray, indices_callback)
    rospy.Subscriber("/disparity_distances", Float32MultiArray, distances_callback)
    rospy.Subscriber("/chosen_gap", Int32, chosen_callback)
    rospy.Subscriber("/chosen_distance", Int32, chosen_distance)
    rospy.Subscriber("/car_0/offboard/command", AckermannDrive, steering_callback)

    rospy.Timer(rospy.Duration(0.1), publish_markers)
    rospy.spin()