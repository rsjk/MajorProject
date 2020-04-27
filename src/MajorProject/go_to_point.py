#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# This code is adapted from Mark Silliman's go_to_specific_point_on_map.py https://github.com/markwsilliman/turtlebot/blob/master/go_to_specific_point_on_map.py

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool

class GoToPose():
    def __init__(self):
        rospy.init_node('go_to_point', anonymous=False)

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
	
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

        self.sub = rospy.Subscriber('user_input', Point, self.callback)
        #self.pub = rospy.Publisher('nav_result', Bool, queue_size=10)

	# Spin until the node is terminated
        rospy.spin()


    def callback(self, data):
        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose = Pose(Point(data.x, data.y, 0.000),
                                     Quaternion(0.000, 0.000, 0.000, 1.000))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False

        # Tell user_input node we've arrived at the destination
        if success:
            rospy.loginfo("We've arrived at the desired destination")
        else:
            rospy.loginfo("The base failed to the desired destination")

        #self.pub.publish(success)
       
        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        GoToPose()
    except rospy.ROSInterruptException:
        rospy.loginfo("go_to_point node terminated")
