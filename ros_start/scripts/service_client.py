#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty

def call_service():
    rospy.loginfo('waiting service')
    rospy.wait_for_service('call_me')
    try:
        service = rospy.ServiceProxy('call_me', Empty)
        response = service()
    except rospy.ServiceException, e:
        print "Service call failed: %s" %e

if __name__ == "__main__":
    for i in range(10):
        call_service()
        rospy.sleep(1.0)
