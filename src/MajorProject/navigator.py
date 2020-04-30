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
# Modifications are indicated in the comments below.

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool


# The Navigator Class listens for a location to send the robot to from the planner node.
class Navigator():
    def __init__(self):
        # Initialize navigator node
        rospy.init_node('navigator', anonymous=False)

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
	
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	rospy.loginfo('Wait for the action server to come up')

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

        # Added these subscibers and publishers
        # Subscriber to get the navigation goal from the planner
        self.sub = rospy.Subscriber('navi_goal', Point, self.callback)
        # Publisher to tell the planner the task is done (still "done" if it failed)
        self.task_pub = rospy.Publisher('navi_done', Bool, queue_size=10)
        # Publisher to tell the planner the result of the task
        self.result_pub = rospy.Publisher('navi_result', Bool, queue_size=10)

	# Spin until the node is terminated
        rospy.spin()


    # Function for when a location from the planner is received (changed from goto to callback for consistency)
    def callback(self, data):
        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        # Changed the first param of pose to data, because I made the planner publish a Point msg
        goal.target_pose.pose = Pose(data,
                                     Quaternion(0.000, 0.000, 0.000, 1.000))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 180 seconds to complete task (increased the time allowed)
	success = self.move_base.wait_for_result(rospy.Duration(180)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False

        # Added these to publish the task results to the planner
        self.result_pub.publish(result)
        self.task_pub.publish(True)
      
        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)


    # Function to perform upon shutdown
    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo('navigator node terminated')
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        # Changed main to simply create a Navigator() instance
        # Removed the hardcoded point and quaternion because instead we listen for a point from the planner
        # Determining success was moved inside the callback function
        Navigator()
    except rospy.ROSInterruptException:
        rospy.loginfo('navigator node terminated')

