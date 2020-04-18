import rospy
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist


# Class to handle collisions. When colliding with an object, the TurtleBot witll halt.
class Collision:
    def __init__(self):
        # initialize
        rospy.init_node('collision')
        
        # Subscribe to the bumper topic
        self.sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.callback)
        
	# Spin until the node is terminated
        rospy.spin()

    # This callback function was written while referencing https://canvas.harvard.edu/courses/37276/pages/getting-started-2-ros-turtlebot-sensors-and-code
    # Modifications are indicated in the comments below
    def callback(self, data):
	# In the original callback function, there was a global variable "bump" set to true if pressed, false otherwise
        if (data.state == BumperEvent.PRESSED):
            rospy.loginfo('Bumper pressed') # Added logging
            rate = rospy.Rate(10)

	    # Added
            move_cmd = Twist()
	    # go backward at 0.2 m/s
            move_cmd.linear.x = -0.2
	    # turn at 0 radians/s
	    move_cmd.angular.z = 0
            self.pub = rospy.Publisher('cmd_vel_mux/input/bumper_halt', Twist, queue_size=10)
            self.pub.publish(move_cmd)
        else:
            rospy.loginfo('Bumper not pressed') # Added logging

	
if __name__ == '__main__':
    try:
        Collision()
    except rospy.ROSInterruptException:
        rospy.loginfo("Collision node terminated.")

