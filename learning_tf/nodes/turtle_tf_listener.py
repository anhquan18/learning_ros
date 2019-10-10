#!/usr/bin/env python
import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('tf_turtle')

    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist, queue_size=1.0)

    rate = rospy.Rate(10.0)
    # Wait until /turtle2 frame load into tf
    listener.waitForTransform("/turtle2", "carrot1", rospy.Time(), rospy.Duration(4.0))
    while not rospy.is_shutdown():
        try:
            #now = rospy.Time.now()
            # Return the transform from source_frame t1 to target_frame t2
            #(trans, rot) = listener.lookupTransform('/turtle2', '/turtle1', rospy.Time(0))
            #(trans, rot) = listener.lookupTransform('/turtle2', "/carrot1", rospy.Time(0)) # get latest available transform
            #(trans, rot) = listener.lookupTransform('/turtle2', "/carrot1", now) # Error will come out because tf's transform isn't load into the listener buffer (usually wait a couple of milliseconds)


            # The waitforTransform take 4 arguments:
            # 1. Wait for the transform from this frame...
            # 2. ...to this frame
            # 3. at this time, and
            # 4. timeout: don't wait for longer than this maximum duration
            #listener.waitForTransform("/turtle2", "/carrot1", now, rospy.Duration(4.0))
            #(trans, rot) = listener.lookupTransform("/turtle2", "/carrot1", now)


            # Turtle2 is going to drive uncontrollably because of the pose of turtle1 5 secs ago relative to turtle2 5 secs ago
            #five_sec_ago = rospy.Time.now() - rospy.Duration(5.0)
            #listener.waitForTransform("/turtle2", "/turtle1", five_sec_ago, rospy.Duration(1.0))
            #(trans, rot) = listener.lookupTransform("/turtle2", "/turtle1", five_sec_ago)


            # THis is how we ask the pose of turtle1 5 secs ago relative to the current turtle2's pose
            now = rospy.Time.now()
            past = now - rospy.Duration(5.0)
            # The lower advanced API for lookupTransform takes 6 arguments
            # 1. Give the transform from this frame
            # 2. at this time ...
            # 3. ... to this frame
            # 4. at this time
            # 5. Specify the frame that does not change over time, in this case the "/world" frame, and
            # 6. the variable to store the result in

            listener.waitForTransformFull("/turtle2", now, "/turtle1", past, "/world", rospy.Duration(2.0))
            (trans, rot) = listener.lookupTransformFull("/turtle2", now,
                                                        "/turtle1", past,
                                                        "/world")

            rospy.loginfo("Transform: "+str(trans))
            rospy.loginfo("Rotation: "+str(rot))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] **2 + trans[1] **2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd)

        rate.sleep()

