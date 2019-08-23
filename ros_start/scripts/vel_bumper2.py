#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent

rospy.init_node('vel_bumper')
vel_x = rospy.get_param('~vevl_x', 0.5)
vel_rot = rospy.get_param('~vel_rot', 1.0)
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

def callback(bumper):
    print bumper
    vel = Twist()
    back_vel.linear.x = -vel_x
    r = rospy.Rate(10.0)
    for i in range(5):
        pub.publish(back_vel)
        r.sleep()
    vel.linear.x = -1.0
    pub.publish(vel)

sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, callback, queue_size=1)

while not rospy.is_shutdown():
    vel = Twist()
    direction = raw_input('f: forward, l: left, r: right, b: back > ')
    if 'f' in direction:
        vel.linear.x = 0.5
    if 'b' in direction:
        vel.linear.x = -0.5
    if 'l' in direction:
        vel.angular.z = 1.0
    if 'r' in direction:
        vel.angular.z = -1.0
    if 'q' in direction:
        break

    print vel
    r = rospy.Rate(10.0)
    for i in range(10):
        pub.publish(vel)
        r.sleep()
