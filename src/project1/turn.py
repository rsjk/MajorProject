#!/usr/bin/env python
import rospy
import math
import random
from math import sqrt
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


# Class to handle turning
# Written using https://answers.ros.org/question/337813/distance-traveled-by-robot-using-odometry/ as reference.
# Modifications are indicated in the comments below.
class TurnHandler:
    def __init__(self):
        # initiliaze
        rospy.init_node('turn')

        # Odometry subscriber and publisher
        self.sub = rospy.Subscriber('odom', Odometry, self.callback)
        self.odom_pub = rospy.Publisher('odom2', Odometry, queue_size=10)
        # Publisher for posting angular velocity (added this to original code)
        self.turn_pub = rospy.Publisher('cmd_vel_mux/input/turn', Twist, queue_size=10)
        
        self.distance = 0.
        self.previous_x = 0
        self.previous_y = 0
        self.first_run = True

        # Spin until node is stopped (added)
        rospy.spin()

    def callback(self, data):
        if self.first_run is True:
            self.previous_x = data.pose.pose.position.x
            self.previous_y = data.pose.pose.position.y
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        d_increment = sqrt((x - self.previous_x) * (x - self.previous_x) + (y - self.previous_y) * (y - self.previous_y))
        self.distance = self.distance + d_increment
        self.odom_pub.publish(data)
        self.previous_x = data.pose.pose.position.x
        self.previous_y = data.pose.pose.position.y
        self.first_run = False
        
        # The original code calculated the total distance traveled. 
        # We added this if statement to check when the robot has moved a foot
	if(self.distance >= 0.3048):
            rospy.loginfo('Turning')
	    self.reset()
	    rospy.sleep(2)
            self.turn_cmd = Twist()
            
	    # Calculate the turn angle
	    if(random.random() > .5):
	        self.turn_cmd.angular.z = .5 # Turn right
	    else:
		self.turn_cmd.angular.z = -.5 # Turn left
            self.turn_pub.publish(self.turn_cmd)
	    rospy.sleep(random.uniform(0.0,.3)) # Degree to turn determined by sleeping an amount sampled from uniform distribution
		
    # Resets distance back to 0 for future sensing (added because orinal code calculated total distance)
    def reset(self):
        self.distance = 0


if __name__ == '__main__':
    try:
        rospy.init_node('turn', anonymous=True)
        odom = TurnHandler()
    except:
        rospy.loginfo('Turn node terminated.')
