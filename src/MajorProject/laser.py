
#! /usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

# When creating code, https://www.theconstructsim.com/read-laserscan-data/ was main reference for getting values from the laser scanner

class LaserHandler:
	def __init__(self):
		#Initialize node
		rospy.init_node('laser')
		#Turn counter to track 180 degree turn for escape behavior
		#Set to high value for start to avoid entering loop immediately 
		self.turnCounter = 1000
		#Set publisher to publish move commands to mid-priority node 
		self.pub = rospy.Publisher('cmd_vel_mux/input/escape_avoid', Twist, queue_size=10)
		#Set up subscriber to get laser scan information
		self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
       		
                #Continue this node until it is quit
		rospy.spin()

	def callback(self, data):
		#Check for objects, along with if they are symetric or asymetric
		#Tracker for what part of the array we are in to see if object is left, right, symmetric, etc
		i = 0
		#Tracks how many points have been discovered in turn range. Needs to surpass a certain threshold to trigger turn
		points = 0
		#Tracks values of close points. Negative = left, positive = right
		tracker = 0
		#Prep movement command 
		self.turn_cmd = Twist()
		#Iterate through laser scan output for object distances
		for dist in data.ranges:
			if dist < 1: # If an object is within a meter, count it as a valid point
				if i < len(data.ranges)/2: #If in the first half of the array, reading is on left. Subtract
					tracker -= dist
				else: #If in the second half of the array, reading is on right. Add
					tracker += dist
				points += 1
			i+=1 #Track current location in the array
		if(self.turnCounter < 35): #If turn counter is below 35, has command to escape
				self.turn_cmd.linear.x = 0 #Set linear speed to 0
				self.turn_cmd.angular.z = .8 #Set turning speed
				self.pub.publish(self.turn_cmd) #Publish movement command
				self.turnCounter += 1 #Keep track of timer
		elif points >= 20: #If not escaping and enough valid points in sight, trigger escape or avoid
			#Layer 2: Avoid asymmetric objects
			if abs(tracker) > 15: #If difference between left and right side is 15, object is asymmetric. Avoid
				self.turn_cmd.linear.x = 0.1 #Set linear speed to slowly forward
				if tracker < 0: #If tracker is negative, object on left. Turn right
					self.turn_cmd.angular.z = 0.5
				else: # If tracker is positive, object on right. Turn left
					self.turn_cmd.angular.z = -0.5
				self.pub.publish(self.turn_cmd) #Send move command
			#Layer 3: Escape symmetric objects
			else: #If not asymmetric, object is symmetric. Set turn counter to 0, triggering 180 turn
				self.turnCounter = 0


if __name__ == '__main__':
       laser = LaserHandler()

	
