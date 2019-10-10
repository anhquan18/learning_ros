#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy 
from geometry_msgs.msg import Twist

class JoyTwist(object):
    def __init__(self):
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=10)
        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.button_x = 0
        self.left_joy_x = 1
        self.left_joy_y = 0

    def joy_callback(self, joy_msg):
        if joy_msg.buttons[self.button_x] == 1:
            print(joy_msg)
            twist = Twist()
            twist.linear.x = joy_msg.axes[self.left_joy_x] *0.5
            twist.angular.z = joy_msg.axes[self.left_joy_y] *1.0
            self.twist_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('joy_twist')
    joy_twist = JoyTwist()
    rospy.spin()
