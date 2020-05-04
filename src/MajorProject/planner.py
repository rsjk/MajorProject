#!/usr/bin/env python

import constant
import math
import rospy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from std_msgs.msg import Bool


# The Planner Class plans locations for the robot to go to. It asks the user whether they want P2P assistance or a tour.
# Upon receiving input, the points to navigate to will be published to the navigator node. Then, it'll listen for the result. 
class Planner():
    def __init__(self):
        # initialize the node
        rospy.init_node('planner')
        
        # Subscriber to get the robot's current pose
        self.sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.poseCallback)
    
        # Publisher to tell navigator where to go
        self.pub = rospy.Publisher('navi_goal', Point, queue_size=10)

        # Subscriber listening for whether navigation task is done
        self.task_sub = rospy.Subscriber('navi_done', Bool, self.taskCallback)
        
        # Subscriber listening for the result of the navigation task
        self.result_sub = rospy.Subscriber('navi_result', Bool, self.resultCallback)

        # Upon launch starting location should be (0, 0)
        self.x_pos = 0.0
        self.y_pos = 0.0

        # Flag for waiting for the robot to reach each location
        self.task_done = False
        
        # Flag for waiting for navigation success
        self.task_result = False
        
        # Run until node is stopped
        while not rospy.is_shutdown():     
            self.userInteraction()

                        
    # Callback function to get the starting pose of the robot    
    def poseCallback(self, data):
        self.x_pos = data.pose.pose.position.x
        self.y_pos = data.pose.pose.position.y    

    
    # Callback function to get the whether a task has completed -- whether it failed or not
    def taskCallback(self, data):
        self.task_done = data
        
        
    # Callback function to get the result of a task    
    def resultCallback(self, data):
        self.task_result = data
        
    
    # Function to get user input on whether they want a P2P or tour. Publishes the plan to navigator and waits for the result.    
    def userInteraction(self):
        print('Would you like to be brought to a location or a tour?:')
        travel_type = raw_input('A: Location\nB: Tour\n') 
 
        # P2P chosen
        if travel_type == 'A':
            print('Please choose one of the following destinations:')
            print('A: {}\nB: {}\nC: {}\nD: {}\nE: {}'.format(constant.ATRIUM_NAME, constant.COMP_LAB_NAME,
                                                               constant.ELEC_LAB_NAME, constant.TEAM_ROOM_NAME, constant.MAIN_OFFICE_NAME))
            location = raw_input()
            point = Point()
            name = ''
            valid = True
 
            # Set what location to go to based on user input
            if location == 'A':
                point = constant.ATRIUM_POS
                name = constant.ATRIUM_NAME
            elif location == 'B':
                point = constant.COMP_LAB_POS
                name = constant.COMP_LAB_NAME
            elif location == 'C':
                point = constant.ELEC_LAB_POS
                name = constant.ELEC_LAB_NAME
            elif location == 'D':
                point = constant.TEAM_ROOM_POS
                name = constant.TEAM_ROOM_NAME
            elif location == 'E':
                point = constant.MAIN_OFFICE_POS
                name = constant.MAIN_OFFICE_NAME
            else:
                # User entered an invalid choice
                print('Invalid choice\n')
                valid = False

            # Publish point if valid
            if valid:
                self.publishPoint(point, name)
                
        # Tour chosen    
        elif travel_type == 'B':
            location = []        # Nearest location. A list of [location name, location point (x, y)]     
            visit_order = []     # Order to visit the locations
            start_x = self.x_pos # Get the robot's current x position
            start_y = self.y_pos # Get the robot's current y position
            
            # Find he nearest location relative to current position
            location = self.findNearestLocation(start_x, start_y) 
            
            # Determine the visit order
            index = 0
            for x in range(len(constant.TOUR_ORDER)):
                if(location[0] == constant.TOUR_ORDER[x]):
                    break
                index += 1
            for x in range(len(constant.TOUR_ORDER)):
                next_loc = [constant.TOUR_ORDER[(x+index)%5], constant.LOCATIONS[constant.TOUR_ORDER[(x+index)%5]]]
                # Append tour location to list of order to visit
                visit_order.append(next_loc)

            # Publish the locations
            for i in range(len(visit_order)):
                self.publishPoint(visit_order[i][1], visit_order[i][0])
        else:
            # User entered an invalid choice
            print('Invalid choice\n')
        
    
    # Function to publish a point to the navigator        
    def publishPoint(self, location, location_name):
        # Publish then wait for the task to be done 
        self.pub.publish(location)    
        while not self.task_done:
            continue
        # Check if navigation was successful
        if self.task_result:
            print("We've arrived at the {}.".format(location_name))
        else:
            print("Sorry, I have failed to bring you to {}.".format(location_name))
        self.task_done = False # Reset to false
                
  
    # Function to find the nearest highlight location to (start_x, start_y)            
    def findNearestLocation(self, start_x, start_y):
        location = Point()
        name = ''
        min_dist = 5000 # Some large number the distance will not be
        # Iterate through tour locations, finding the distance
        for i in constant.TOUR_ORDER:
            # sqrt((x_a - x_b)^2 + (y_a - y_b)^2)
            dist =  math.sqrt((start_x - constant.LOCATIONS[i].x)**2 + (start_y - constant.LOCATIONS[i].y)**2)
            # If current dist is less than min dist, replace min dist
            if dist < min_dist:
                min_dist = dist
                location = constant.LOCATIONS[i] # Get the point to go to
                name = i # Get the name of the highlight
        location_list = [name, location]
        return location_list # Return the selected location and name of the location

      
if __name__ == '__main__':
    try:
        Planner()   
    except rospy.ROSInterruptException:
        rospy.loginfo('planner node terminated')

