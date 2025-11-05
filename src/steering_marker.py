#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Int32
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler


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
    if 0 <= chosen_idx < len(distances):
        angle = ANGLE_MIN + chosen_idx * ANGLE_INCREMENT
        dist = distances[chosen_idx]
        x = dist * math.cos(angle)
        y = dist * math.sin(angle)
        target = Marker()
        target.header.frame_id = "car_0_base_link"
        target.header.stamp = rospy.Time.now()
        target.ns = "target_point"
        target.id = 9999
        target.type = Marker.SPHERE
        target.action = Marker.ADD
        target.pose.position.x = x 
        target.pose.position.y = y
        target.pose.position.z = 0.2
        target.scale.x = 0.25
        target.scale.y = 0.25
        target.scale.z = 0.25
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

    # Place arrow slightly in front of the car
    m.pose.position.x = 0.5
    m.pose.position.y = 0.0
    m.pose.position.z = 0.2

    # Your controller publishes steering_angle in **degrees** (you multiplied by 1.5 there),
    # so convert degrees -> radians for the marker quaternion.
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
   
    rospy.Subscriber("/car_0/offboard/command", AckermannDrive, steering_callback)

    rospy.Timer(rospy.Duration(0.2), publish_markers)
    rospy.spin()
