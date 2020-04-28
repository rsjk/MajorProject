#!/usr/bin/env python

import rospy
import tf

if __name__ == 'main':
    rospy.init_node('lisenter')

    listener = tf.TransformListener()
    '''
    tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
    transformed_pose_stamped = tf_listener.transformPose('map', pose_stamped)
    print(transformed_pose_stamped)
    '''
    print('hello')
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('exception occured')
            continue
    
