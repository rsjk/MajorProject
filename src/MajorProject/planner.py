#!/usr/bin/env python

import constant
import math
import rospy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from std_msgs.msg import Bool


class Planner():
    def __init__(self):
        rospy.init_node('planner')
        # Subscriber to get the robot's current pose
        self.sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.poseCallback)
    
        # Publisher to tell navigator where to go
        self.pub = rospy.Publisher('navi_goal', Point, queue_size=10)

        # Subscriber listening for whether navigation task is done
        self.sub_2 = rospy.Subscriber('navi_done', Bool, self.taskCallback)
        
        # Subscriber listening for whether navigation task was successful
        self.sub_3 = rospy.Subscriber('navi_result', Bool, self.resultCallback)

        # Upon launch starting location should be (0, 0)
        self.x_pos = 0.0
        self.y_pos = 0.0

        # Flag for waiting for the robot to reach each location
        self.task_done = False
        
        # Flag for waiting for navigation success
        self.task_result = False
        
        while not rospy.is_shutdown():     
            self.userInteraction()

                        
    # Function to get the starting pose of the robot    
    def poseCallback(self, data):
        self.x_pos = data.pose.pose.position.x
        self.y_pos = data.pose.pose.position.y    

    
    # Function to get the whether a task has completed -- whether it failed or not
    def taskCallback(self, data):
        self.task_done = data
        
        
    # Function to get the result of a task    
    def resultCallback(self, data):
        self.task_result = data
        
    
    # Function to get user input on whether they want a P2P or tour. Publishes the plan to navigator and waits for the result.    
    def userInteraction(self):
        print("Hello! Would you like to be brought to a location or a tour?:")
        travel_type = raw_input("A: Location\nB: Tour\n") 
 
        # P2P chosen
        if travel_type == 'A':
            print("Please choose one of the following destinations:")
            print("A: {}\nB: {}\nC: {}\nD: {}\nE: {}".format(constant.ATRIUM_NAME, constant.COMP_LAB_NAME,
                                                               constant.ELEC_LAB_NAME, constant.TEAM_ROOM_NAME, constant.MAIN_OFFICE_NAME))
            location = raw_input()
            point = Point()
            name = ''
            valid = True
 
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
                print('Invalid choice\n')
                valid = False

            # Publish point if valid
            if valid:
                '''
                self.pub.publish(point)
                while not self.task_done:
                    continue
                self.task_done = False # Reset to false
                
                # Check if navigation was successful
                if self.task_result:
                    print("We've arrived at {}.".format(name))
                '''
                self.publishPoint(point, name)
                
        # Tour chosen    
        elif travel_type == 'B':
            # Calculate nearest point
            # sqrt((x_a - x_b)^2 + (y_a - y_b)^2)
            remaining_locations = [constant.ATRIUM_NAME, constant.COMP_LAB_NAME, constant.ELEC_LAB_NAME, constant.TEAM_ROOM_NAME, constant.MAIN_OFFICE_NAME]
            
            tour_point = Point()
            visit_order = []
            for x in range(len(remaining_locations)):
                if x == 0:
                    # Tour just started -- set position to position determined by poseCallback
                    start_x = self.x_pos
                    start_y = self.y_pos
                else:
                    # Next position to find nearest location is the tour_point previously found
                    start_x = tour_point[1].x
                    start_y = tour_point[1].y
                    
                tour_point = self.findNearestLocation(start_x, start_y, remaining_locations) 
                # Append tour point to list of order to visit
                visit_order.append(tour_point)    

            # Publish the points
            for i in range(len(visit_order)):
                print(i)
                self.publishPoint(visit_order[i][1], visit_order[i][0])
                
            '''    
            # Publish then wait for the task to be done
            self.pub.publish(visit_order[0][1])
            while not self.task_done:
                continue  
            # Check if navigation was successful
            if self.task_result:
                print("We've arrived at the {}.".format(visit_order[0][0]))
            self.task_done = False # Reset to false
            
            # Publish then wait for the task to be done    
            self.pub.publish(visit_order[1][1])    
            while not self.task_done:
                continue
            # Check if navigation was successful
            if self.task_result:
                print("We've arrived at the {}.".format(visit_order[1][0]))
            self.task_done = False # Reset to false
            
            # Publish then wait for the task to be done    
            self.pub.publish(visit_order[2][1])    
            while not self.task_done:
                continue
            # Check if navigation was successful
            if self.task_result:
                print("We've arrived at the {}.".format(visit_order[2][0]))
            self.task_done = False # Reset to false 
            
            # Publish then wait for the task to be done    
            self.pub.publish(visit_order[3][1])    
            while not self.task_done:
                continue
            # Check if navigation was successful
            if self.task_result:
                print("We've arrived at {}.".format(visit_order[3][0]))
            self.task_done = False # Reset to false
            
            # Publish then wait for the task to be done    
            self.pub.publish(visit_order[4][1])    
            while not self.task_done:
                continue
            # Check if navigation was successful
            if self.task_result:
                print("We've arrived at the {}.".format(visit_order[4][0]))
            self.task_done = False # Reset to false
            '''
        else:
            print('Invalid choice\n')
        
            
    def publishPoint(self, location, location_name):
        # Publish then wait for the task to be done 
        self.pub.publish(location)    
        while not self.task_done:
            continue
        # Check if navigation was successful
        if self.task_result:
            print("We've arrived at the {}.".format(location_name))
        self.task_done = False # Reset to false
                
  
      
    # Function to find the nearest highlight location to (start_x, start_y)            
    def findNearestLocation(self, start_x, start_y, remaining_locations):
        location = Point()
        name = ''
        min_dist = 1000 # Some large number the distance will not be
        # Iterate through remaining locations, finding the distance
        for x in remaining_locations:
            # sqrt((x_a - x_b)^2 + (y_a - y_b)^2)
            dist =  math.sqrt((start_x - constant.LOCATIONS[x].x)**2 + (start_y - constant.LOCATIONS[x].y)**2)
            # If current dist is less than min dist, replace min dist
            if dist < min_dist:
                min_dist = dist
                location = constant.LOCATIONS[x]
                name = x     
        remaining_locations.remove(name) # Remove selected location from the list of remaining locations
        location_list = [name, location]
        return location_list # Return the selected location

      
if __name__ == '__main__':
    try:
        Planner()   
    except rospy.ROSInterruptException:
        rospy.loginfo('planner node terminated')

