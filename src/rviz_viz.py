#!/usr/bin/env python
import math
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
from race.msg import pid_input
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class RVizFTG:
    def __init__(self):
        # Params
        self.frame_id       = rospy.get_param("~frame_id", "base_link")  # or "laser"
        self.target_radius  = rospy.get_param("~target_radius", 1.5)
        self.disp_thresh    = rospy.get_param("~disparity_threshold", 0.18)
        self.max_disp_mark  = rospy.get_param("~max_disparity_markers", 250)
        self.arrow_len      = rospy.get_param("~arrow_length", 1.0)
        self.pub_rate_hz    = rospy.get_param("~pub_rate_hz", 10)

        # State
        self.target_angle = 0.0
        self.have_target  = False
        self.steer_angle  = 0.0
        self.scan         = None

        # Publishers
        self.gap_pub   = rospy.Publisher("/gap_markers", MarkerArray, queue_size=1)
        self.steer_pub = rospy.Publisher("/steer_marker", Marker, queue_size=1)

        # Subscribers
        rospy.Subscriber("error", pid_input, self.cb_target, queue_size=1)
        rospy.Subscriber("/car_0/scan", LaserScan, self.cb_scan, queue_size=1)
        rospy.Subscriber("/car_0/offboard/command", AckermannDrive, self.cb_cmd, queue_size=1)

        # Timer
        rospy.Timer(rospy.Duration(1.0/self.pub_rate_hz), self.on_timer)

    def cb_target(self, msg):
        ang = float(msg.pid_error)
        if abs(ang) > 2.0*math.pi:  # auto-convert if degrees appear
            ang = math.radians(ang)
        self.target_angle = ang
        self.have_target  = True

    def cb_scan(self, msg):
        self.scan = msg

    def cb_cmd(self, msg):
        ang = float(getattr(msg, "steering_angle", 0.0))
        if abs(ang) > 2.0*math.pi:
            ang = math.radians(ang)
        # normalize to [-pi, pi]
        while ang >  math.pi: ang -= 2.0*math.pi
        while ang < -math.pi: ang += 2.0*math.pi
        self.steer_angle = ang

    def on_timer(self, _evt):
        self.publish_steer_arrow()
        self.publish_gap_markers()

    # ----- Steering (Marker.ARROW built from two points; no spinning) -----
    def publish_steer_arrow(self):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp    = rospy.Time.now()
        m.ns = "steering"
        m.id = 1
        m.type = Marker.ARROW
        m.action = Marker.ADD

        p0 = Point(0.0, 0.0, 0.0)
        L  = self.arrow_len
        p1 = Point(L*math.cos(self.steer_angle), L*math.sin(self.steer_angle), 0.0)
        m.points = [p0, p1]

        # For ARROW-with-points: scale.x = shaft dia, scale.y = head dia
        m.scale.x = 0.05
        m.scale.y = 0.08
        m.scale.z = 0.0
        m.color.r, m.color.g, m.color.b, m.color.a = (0.1, 0.4, 0.9, 0.95)
        self.steer_pub.publish(m)

    # ----- Target sphere + disparities (MarkerArray) -----
    def publish_gap_markers(self):
        if self.scan is None:
            return

        now = rospy.Time.now()
        ma = MarkerArray()

        # Target sphere at chosen gap angle
        if self.have_target:
            mt = Marker()
            mt.header.frame_id = self.frame_id
            mt.header.stamp    = now
            mt.ns = "gap"
            mt.id = 1
            mt.type = Marker.SPHERE
            mt.action = Marker.ADD
            R = self.target_radius
            mt.pose.position.x = R*math.cos(self.target_angle)
            mt.pose.position.y = R*math.sin(self.target_angle)
            mt.pose.position.z = 0.0
            mt.pose.orientation.w = 1.0
            mt.scale.x = mt.scale.y = mt.scale.z = 0.15
            mt.color.r, mt.color.g, mt.color.b, mt.color.a = (0.1, 0.9, 0.1, 0.9)
            ma.markers.append(mt)

        # Disparity cubes (adjacent range jump > threshold)
        idxs = self._disparity_indices(self.scan.ranges, self.disp_thresh)
        cnt = 0
        for i in idxs:
            if cnt >= self.max_disp_mark:
                break
            d = self.scan.ranges[i]
            if not (math.isfinite(d) and d > 0.0):
                continue
            ang = self.scan.angle_min + i*self.scan.angle_increment

            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp    = now
            m.ns = "disp"
            m.id = 1000 + cnt
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = d*math.cos(ang)
            m.pose.position.y = d*math.sin(ang)
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.06
            m.scale.z = 0.02
            m.color.r, m.color.g, m.color.b, m.color.a = (0.9, 0.1, 0.1, 0.9)
            ma.markers.append(m)
            cnt += 1

        if ma.markers:
            self.gap_pub.publish(ma)

    @staticmethod
    def _disparity_indices(ranges, thresh):
        idxs = []
        for i in range(len(ranges) - 1):
            a, b = ranges[i], ranges[i+1]
            if (math.isfinite(a) and a > 0.0) and (math.isfinite(b) and b > 0.0):
                if abs(a - b) > thresh:
                    idxs.append(i if a < b else i+1)
        return idxs

if __name__ == "__main__":
    rospy.init_node("rviz_viz", anonymous=True)
    RVizFTG()
    rospy.spin()
