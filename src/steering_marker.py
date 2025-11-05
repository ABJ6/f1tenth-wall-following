#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Int32
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import LaserScan

# ---------------- Parameters / Constants (kept) ----------------
ANGLE_MIN = -math.radians(90)                 # used for disparity dots fallback
ANGLE_INCREMENT = math.radians(0.3515625)     # used for disparity dots fallback
FRAME_ID = "car_0_base_link"

# ---------------- Publishers ----------------
marker_pub = rospy.Publisher('/disparity_markers', Marker, queue_size=10)

# ---------------- State ----------------
indices = []          # disparity indices
distances = []        # disparity distances
chosen_idx = -1       # chosen gap index (reduced-list index or lidar bin)
last_scan = None      # live LaserScan (for correct angle_min/angle_increment)

# ---------------- Callbacks ----------------
def indices_callback(msg):
    global indices
    indices = msg.data

def distances_callback(msg):
    global distances
    distances = msg.data

def chosen_callback(msg):
    global chosen_idx
    chosen_idx = msg.data

def steering_callback(cmd):
    # Draw the steering arrow the same way you had (orientation-based)
    m = Marker()
    m.header.frame_id = FRAME_ID
    m.header.stamp    = rospy.Time.now()
    m.ns   = "steering"
    m.id   = 1
    m.type = Marker.ARROW
    m.action = Marker.ADD

    # place arrow slightly in front, keep your look
    m.pose.position.x = 0.5
    m.pose.position.y = 0.0
    m.pose.position.z = 0.2

    yaw_val = getattr(cmd, "steering_angle", 0.0)
    # auto-convert if degrees slipped in
    yaw = math.radians(yaw_val) if abs(yaw_val) > 2.0 * math.pi else float(yaw_val)
    q = quaternion_from_euler(0.0, 0.0, yaw)
    m.pose.orientation = Quaternion(*q)

    m.scale.x = 0.6   # shaft length
    m.scale.y = 0.05  # shaft diameter
    m.scale.z = 0.08  # head diameter
    m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.4, 0.0, 1.0
    m.lifetime = rospy.Duration(0.25)

    marker_pub.publish(m)

def scan_callback(msg):
    # Cache the live scan so target sphere uses correct angle_min / angle_increment
    global last_scan
    last_scan = msg

# ---------------- Timer: publish disparities + target sphere ----------------
def publish_markers(_event):
    # 1) Disparity dots (yellow), keep your original look/format
    for i, (idx, dist) in enumerate(zip(indices, distances)):
        # default to your constants, but if we have a live scan, prefer its metadata
        if last_scan is not None:
            angle_min = float(last_scan.angle_min)
            angle_inc = float(last_scan.angle_increment)
        else:
            angle_min = ANGLE_MIN
            angle_inc = ANGLE_INCREMENT

        lidar_idx = int(idx)
        if not (math.isfinite(dist) and dist > 0.0):
            continue

        ang = angle_min + lidar_idx * angle_inc
        x = dist * math.cos(ang)
        y = dist * math.sin(ang)

        m = Marker()
        m.header.frame_id = FRAME_ID
        m.header.stamp = rospy.Time.now()
        m.ns = "disparity"
        m.id = 200 + i
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.2
        m.scale.x = m.scale.y = m.scale.z = 0.13

        if i == chosen_idx:
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
        else:
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 1.0, 0.0, 1.0

        marker_pub.publish(m)

    # 2) Target point (green sphere) — UPDATED: use live scan’s angle_min/angle_increment
    if 0 <= chosen_idx < len(distances) and (last_scan is not None):
        # If chosen_idx is reduced-list index, map it; if it’s already a lidar bin, this still works
        lidar_idx_for_target = int(indices[chosen_idx]) if len(indices) > chosen_idx else int(chosen_idx)

        angle_min_live = float(last_scan.angle_min)
        angle_inc_live = float(last_scan.angle_increment)
        ang_t  = angle_min_live + lidar_idx_for_target * angle_inc_live

        dist_t = float(distances[chosen_idx])
        if not (math.isfinite(dist_t) and dist_t > 0.0):
            dist_t = 1.5  # safe fallback radius

        tx = dist_t * math.cos(ang_t)
        ty = dist_t * math.sin(ang_t)

        target = Marker()
        target.header.frame_id = FRAME_ID
        target.header.stamp = rospy.Time.now()
        target.ns = "target_point"
        target.id = 9999
        target.type = Marker.SPHERE
        target.action = Marker.ADD
        target.pose.position.x = tx
        target.pose.position.y = ty
        target.pose.position.z = 0.2
        target.scale.x = 0.25
        target.scale.y = 0.25
        target.scale.z = 0.25
        target.color.r = 0.0
        target.color.g = 1.0
        target.color.b = 0.0
        target.color.a = 1.0
        target.lifetime = rospy.Duration(0.2)  # keeps the sphere visually fresh
        marker_pub.publish(target)

# ---------------- Main ----------------
if __name__ == '__main__':
    rospy.init_node("disparity_marker_visualizer")

    rospy.Subscriber("/disparity_indices",   Int32MultiArray,   indices_callback)
    rospy.Subscriber("/disparity_distances", Float32MultiArray, distances_callback)
    rospy.Subscriber("/chosen_gap",          Int32,             chosen_callback)
    rospy.Subscriber("/car_0/offboard/command", AckermannDrive, steering_callback)
    rospy.Subscriber("/car_0/scan", LaserScan, scan_callback)

    rospy.Timer(rospy.Duration(0.2), publish_markers)
    rospy.spin()
