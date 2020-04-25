#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PointStamped

def transformPoint(listener):
    laser_point = PointStamped()
    laser_point.header.frame_id = 'base_laser'
    
    laser_point.header.stamp = rospy.Time()
    
    laser_point.point.x = 1.0
    laser_point.point.y = 0.2
    laser_point.point.z = 0.0
    
    try:
        base_point = PointStamped()
        listener.transformPoint('base_link', laser_point, base_point)
        
        rospy.loginfo("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f", laser_point.point.x, laser_point.point.y, laser_point.point.z, base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec())
        
        
    except tf.Exception:
        rospy.logerr('Exception trying to transform a point from "base_laser" to "base_link"')
        
if __name__ == '__main__':
    rospy.init_node('robot_tf_listener')
    
    listener = tf.TransformListener(rospy.Duration(10))
    
    while not rospy.is_shutdown():
        transformPoint(listener)
        sleep(1)