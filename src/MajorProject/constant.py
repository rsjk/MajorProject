#!/usr/bin/env python

from geometry_msgs.msg import Point


# Highligh names
ATRIUM_NAME = 'Atrium'
COMP_LAB_NAME = 'Computer Lab'
ELEC_LAB_NAME = 'Electronics Lab'
TEAM_ROOM_NAME = 'Team Room'
MAIN_OFFICE_NAME = 'Main Office'

# Highlight locations
ATRIUM_POS = Point(2, -5, 0)
COMP_LAB_POS = Point(8, -1, 0)
ELEC_LAB_POS = Point(1, -1, 0)
TEAM_ROOM_POS = Point(-8, -6, 0)
MAIN_OFFICE_POS = Point(-8, 3, 0)

# Highlight locations bounding points
ATRIUM_BOUNDS = (Point(-3.67, -7.06, 0), Point(9.62, -2.52, 0))
COMP_LAB_BOUNDS = (Point(5.47, -2.46, 0), Point(9.96, 1.05, 0))
ELEC_LAB_BOUNDS = (Point(-3.41, -2.96, 0), Point(5.26, 0.802, 0))
TEAM_ROOM_BOUNDS = (Point(-9.92, -6.99, 0), Point(-6.53, -4.47, 0))
MAIN_OFFICE_BOUNDS = (Point(-9.95, 0.514, 0), Point(-6.6, 4.86, 0))

# Dictionary of locations
LOCATIONS = {ATRIUM_NAME: ATRIUM_POS, COMP_LAB_NAME : COMP_LAB_POS, ELEC_LAB_NAME: ELEC_LAB_POS, 
                              TEAM_ROOM_NAME: TEAM_ROOM_POS, MAIN_OFFICE_NAME: MAIN_OFFICE_POS}

# Pre-determined tour route. If starting location is closest to the electronics lab, next location will be the team room and follow the order.
TOUR_ORDER = [TEAM_ROOM_NAME, ATRIUM_NAME, COMP_LAB_NAME, MAIN_OFFICE_NAME, ELEC_LAB_NAME]