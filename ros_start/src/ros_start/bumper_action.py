#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
import actionlib
from ros_start.msg import GoUntilBumperAction
from ros_start.msg import GoUntilBumperResult
from ros_start.msg import GoUntilBumperFeedback

class BumperAction(object):
    def __init__(self):
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist,
                                   queue_size=10)
        self.sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent,
                                    self.bumper_callback, queue_size=1)
        self.max_vel = rospy.get_param('~max_vel', 0.5)
        self.action_server = actionlib.SimpleActionServer(
                'bumper_action', GoUntilBumperAction, 
                execute_cb=self.go_until_bumper, auto_start=False)
        self.hit_bumper = False
        self.action_server.start()

    def bumper_callback(self, bumper):
        self.hit_bumper = True

    def go_until_bumper(self, goal):
        print(goal.target_vel)
        r = rospy.Rate(10.0)
        zero_vel = Twist()
        for i in range(10 * goal.timeout_sec):
            if self.action_server.is_preempt_requested():# Request stop from outside
                self.action_server.set_preempted() # Stop action
                break
            if self.hit_bumper:
                self.pub.publish(zero_vel)
                break
            else:
                if goal.target_vel.linear.x > self.max_vel:
                    goal.target_vel.linear.x = self.max_vel
                self.pub.publish(goal.target_vel)
                feedback = GoUntilBumperFeedback(current_vel=goal.target_vel)
                self.action_server.publish_feedback(feedback)
            r.sleep()
        result = GoUntilBumperResult(bumper_hit=self.hit_bumper)
        self.action_server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('bumper_action')
    bumper_action = BumperAction()
    rospy.spin()
