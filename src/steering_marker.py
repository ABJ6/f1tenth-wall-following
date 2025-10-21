#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDrive
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler


CMD_TOPIC  = '/car_0/offboard/command'   
FRAME_ID   = 'car_0_base_link'                 
NS         = 'steering'
ARROW_ID   = 1

POS_X      = 0.5
POS_Y      = 0.0
POS_Z      = 0.2
SHAFT_LEN  = 0.6
SHAFT_DIAM = 0.05
HEAD_DIAM  = 0.08


COLOR_R    = 1.0
COLOR_G    = 0.4
COLOR_B    = 0.0
COLOR_A    = 1.0

marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)


def control(data):
    """Subscribe to AckermannDrive and draw an arrow for steering."""
    m = Marker()
    m.header.frame_id = FRAME_ID
    m.header.stamp    = rospy.Time.now()
    m.ns   = NS
    m.id   = ARROW_ID
    m.type = Marker.ARROW
    m.action = Marker.ADD

    
    m.pose.position.x = POS_X
    m.pose.position.y = POS_Y
    m.pose.position.z = POS_Z

    
    yaw = getattr(data, 'steering_angle', 0.0)
    q = quaternion_from_euler(0.0, 0.0, yaw)
    m.pose.orientation = Quaternion(*q)

    
    m.scale.x = SHAFT_LEN
    m.scale.y = SHAFT_DIAM
    m.scale.z = HEAD_DIAM
    m.color.r = COLOR_R
    m.color.g = COLOR_G
    m.color.b = COLOR_B
    m.color.a = COLOR_A

    marker_pub.publish(m)

if __name__ == '__main__':
    rospy.init_node('steering_marker', anonymous=True)
    rospy.Subscriber(CMD_TOPIC, AckermannDrive, control)
    rospy.spin()
