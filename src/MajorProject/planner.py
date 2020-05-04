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
            destination = raw_input()
            target_loc = Point()
            target_name = ''
            valid = True
 
            # Set what destination to go to based on user input
            if destination == 'A':
                target_loc = constant.ATRIUM_POS
                target_name = constant.ATRIUM_NAME
            elif destination == 'B':
                target_loc = constant.COMP_LAB_POS
                target_name = constant.COMP_LAB_NAME
            elif destination == 'C':
                target_loc = constant.ELEC_LAB_POS
                target_name = constant.ELEC_LAB_NAME
            elif destination == 'D':
                target_loc = constant.TEAM_ROOM_POS
                target_name = constant.TEAM_ROOM_NAME
            elif destination == 'E':
                target_loc = constant.MAIN_OFFICE_POS
                target_name = constant.MAIN_OFFICE_NAME
            else:
                # User entered an invalid choice
                print('Invalid choice\n')
                valid = False
                
            # Get current position
            start_x = self.x_pos
            start_y = self.y_pos

            # Check if the robot is already in the location
            in_location = self.inLocation(start_x, start_y, target_loc)
            
            # Publish point if valid and not in location
            if in_location: 
                print("We're at the {}.".format(target_name))
            else:
                if valid:
                    self.publishPoint(target_loc, target_name)
                
        # Tour chosen    
        elif travel_type == 'B': 
            visit_order = []     # Order to visit the locations
            start_x = self.x_pos # Get the robot's current x position
            start_y = self.y_pos # Get the robot's current y position
            
            # Find he nearest location relative to current position
            nearest_loc = self.findNearestLocation(start_x, start_y)
            
            # Check if the robot is already in the location
            in_location = self.inLocation(start_x, start_y, nearest_loc[1])
            
            # Find index of location in tour order
            index = 0
            for x in range(len(constant.TOUR_ORDER)):
                if(nearest_loc[0] == constant.TOUR_ORDER[x]):
                    break
                index += 1
            # Determine the visit order
            for x in range(len(constant.TOUR_ORDER)):
                next_loc = [constant.TOUR_ORDER[(x+index)%5], constant.LOCATIONS[constant.TOUR_ORDER[(x+index)%5]]]
                # Append tour location to list of order to visit
                if not in_location:
                    visit_order.append(next_loc)
                else:
                    # Already in location -- skip publishing first point
                    if x == 0:
                        print("We're in the {}.".format(next_loc[0]))
                    else:
                        visit_order.append(next_loc)

            # Publish the locations
            for i in range(len(visit_order)):
                self.publishPoint(visit_order[i][1], visit_order[i][0])
        else:
            # User entered an invalid choice
            print('Invalid choice\n')
 
 
    # Function to determine if the current position is already in the given location
    def inLocation(self, start_x, start_y, location):
        if location == constant.ATRIUM_POS:
            if constant.ATRIUM_BOUNDS[0].x <= start_x <= constant.ATRIUM_BOUNDS[1].x:
                if constant.ATRIUM_BOUNDS[0].y <= start_y <= constant.ATRIUM_BOUNDS[1].y:
                    # Current location is in atrium bounds
                    return True
        elif location == constant.COMP_LAB_POS:
            if constant.COMP_LAB_BOUNDS[0].x <= start_x <= constant.COMP_LAB_BOUNDS[1].x:
                if constant.COMP_LAB_BOUNDS[0].y <= start_y <= constant.COMP_LAB_BOUNDS[1].y:
                    # Current location is in computer labbounds
                    return True
        elif location == constant.ELEC_LAB_POS:
            if constant.ELEC_LAB_BOUNDS[0].x <= start_x <= constant.ELEC_LAB_BOUNDS[1].x:
                if constant.ELEC_LAB_BOUNDS[0].y <= start_y <= constant.ELEC_LAB_BOUNDS[1].y:
                    # Current location is in electric lab bounds
                    return True           
        elif location == constant.TEAM_ROOM_POS:
            if constant.TEAM_ROOM_BOUNDS[0].x <= start_x <= constant.TEAM_ROOM_BOUNDS[1].x:
                if constant.TEAM_ROOM_BOUNDS[0].y <= start_y <= constant.TEAM_ROOM_BOUNDS[1].y:
                    # Current location is in team room bounds
                    return True
        elif location == constant.MAIN_OFFICE_POS:
            if constant.MAIN_OFFICE_BOUNDS[0].x <= start_x <= constant.MAIN_OFFICE_BOUNDS[1].x:
                if constant.MAIN_OFFICE_BOUNDS[0].y <= start_y <= constant.MAIN_OFFICE_BOUNDS[1].y:
                    # Current location is in main office bounds
                    return True
 
 
    # Function to find the nearest highlight location to (start_x, start_y)  
    def findNearestLocation(self, start_x, start_y):
        location = ()
        min_dist = 5000 # Some large number the distance will not be
        for item in constant.LOCATIONS.items():
            # sqrt((x_a - x_b)^2 + (y_a - y_b)^2)
            dist =  math.sqrt((start_x - item[1].x)**2 + (start_y - item[1].y)**2)
            # If current dist is less than min dist, replace min dist
            if dist < min_dist:
                min_dist = dist
                location = item
        return location  
           
    
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

      
if __name__ == '__main__':
    try:
        Planner()   
    except rospy.ROSInterruptException:
        rospy.loginfo('planner node terminated')

