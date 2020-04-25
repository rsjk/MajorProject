#!/usr/bin/env python
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('robot_tf_publisher')
    
    r = rospy.Rate(100)
    broadcaster = tf.TransformBroadcaster();
    
    while not rospy.is_shutdown():
        '''
        broadcaster.sendTransform(tf.TransformStamped(tf.Transform(tf.Quaternion(0, 0, 0, 1), tf.Vector3(0.1, 0.0, 0.2)), rospyTime.now(),'base_link', 'base_laser'))
        '''
        broadcaster.sendTransform(

        r.sleep();

