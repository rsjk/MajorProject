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

# Dictionary of locations
LOCATIONS = {ATRIUM_NAME: ATRIUM_POS, COMP_LAB_NAME : COMP_LAB_POS, ELEC_LAB_NAME: ELEC_LAB_POS, 
                              TEAM_ROOM_NAME: TEAM_ROOM_POS, MAIN_OFFICE_NAME: MAIN_OFFICE_POS}