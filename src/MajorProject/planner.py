import math
import rospy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from std_msgs.msg import Bool


class Location():
    def __init__(self, name='Name', point=Point(0, 0, 0)):
        self.name = name
        self.point = point
        
    def getName(self):
        return self.name
    
    def getPoint(self): 
        return self.point
    
    def getX(self):
        return self.point.x
    
    def getY(self):
        return self.point.y
    
    def setName(self, name):
        self.name = name
        
    def setPoint(self, point):
        self.point = point


class Planner():
    def __init__(self):
        rospy.init_node('planner')
        # Subscriber to get the robot's current pose
        self.sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.poseCallback)
    
        # Publisher to tell navigator where to go
        self.pub = rospy.Publisher('navi_goal', Point, queue_size=10)

        # Subsriber listening for navigation result
        self.sub_2 = rospy.Subscriber('navi_result', Bool, self.taskCallback)

        # Upon launch starting location should be (0, 0)
        self.x_pos = 0.0
        self.y_pos = 0.0

        # Flag for waiting for the robot to reach each location
        self.task_done = False
        
        while not rospy.is_shutdown():
            # Set the highlight locations
            atrium = Point(2, -5, 0)
            computer_lab = Point(8, -1, 0)
            electronics_lab = Point(1, -1, 0)
            team_room = Point(-8, -6, 0)
            main_office = Point(-8, 3, 0)    
            
            # A dictionary of the locations
            location_dict = {'Atrium': atrium, 'Computer Lab': computer_lab, 'Electronics Lab': electronics_lab, 
                              'Team Room': team_room, 'Main Office': main_office}                   
            self.getUserInput(location_dict)

                        
    # Function to get the starting pose of the robot    
    def poseCallback(self, data):
        self.x_pos = data.pose.pose.position.x
        self.y_pos = data.pose.pose.position.y    

    
    def taskCallback(self, data):
        self.task_done = data
        
    def getUserInput(self, location_dict):
        print("Hello! Would you like to be brought to a location or a tour?:")
        travel_type = raw_input("A: Location\nB: Tour\n") 
        
        point = Point()
        valid = True

        # P2P chosen
        if travel_type == 'A':
            print("Please choose one of the following destinations:")
            location = raw_input("A: Atrium\nB: Computer Lab\nC: Electronics Lab\nD: Team Room\nE: Main Office\n")

            if location == 'A':
                point = location_dict['Atrium']
            elif location == 'B':
                point = location_dict['Computer Lab']
            elif location == 'C':
                point = location_dict['Electronics Lab']
            elif location == 'D':
                point = location_dict['Team Room']
            elif location == 'E':
                point = location_dict['Main Office']
            else:
                print('Invalid choice\n')
                valid = False

            # Publish point if valid
            if valid:
                self.pub.publish(point)
                while not self.task_done:
                    continue
                self.task_done = False # Reset to false
        # Tour chosen    
        elif travel_type == 'B':
            # Calculate nearest point
            # sqrt((x_a - x_b)^2 + (y_a - y_b)^2)
            remaining_locations = ['Atrium', 'Computer Lab', 'Electronics Lab', 'Team Room', 'Main Office']
            
            tour_point = Point()
            visit_order = []
            for x in range(len(remaining_locations)):
                if x == 0:
                    # Tour just started -- set position to position determined by poseCallback
                    start_x = self.x_pos
                    start_y = self.y_pos
                else:
                    # Next position to find nearest location is the tour_point previously found
                    start_x = tour_point.x
                    start_y = tour_point.y
                    
                tour_point = self.findNearestLocation(start_x, start_y, location_dict, remaining_locations) 
                # Append tour point to list of order to visit
                visit_order.append(tour_point)    


            # Wait for task_done flag to be set before publishing next velocity
            while not self.task_done:
                continue
            self.pub.publish(visit_order[0])
            self.task_done = False 
            while not self.task_done:
                continue
            self.pub.publish(visit_order[1])
            self.task_done = False 
            while not self.task_done:
                continue
            self.pub.publish(visit_order[2])
            self.task_done = False 
            while not self.task_done:
                continue
            self.pub.publish(visit_order[3])
            self.task_done = False   
            while not self.task_done:
                continue
            self.pub.publish(visit_order[4])
            self.task_done = False      
            while not self.task_done:
                continue
            self.task_done = False # Reset to false 
        else:
            print('Invalid choice\n')    
                
    def findNearestLocation(self, start_x, start_y, location_dict, remaining_locations):
        location = Point()
        name = ''
        min_dist = 1000 # Some large number the distance will not be
        # Iterate through remaining locations, finding the distance
        for x in remaining_locations:
            # sqrt((x_a - x_b)^2 + (y_a - y_b)^2)
            dist =  math.sqrt((start_x - location_dict[x].x)**2 + (start_y - location_dict[x].y)**2)
            # If current dist is less than min dist, replace min dist
            if dist < min_dist:
                min_dist = dist
                location = location_dict[x]
                name = x     
        remaining_locations.remove(name) # Remove selected location from the list of remaining locations
        return location # Return the selected location

        
        
if __name__ == '__main__':
    try:
        Planner()   
    except rospy.ROSInterruptException:
        rospy.loginfo('planner node terminated')
