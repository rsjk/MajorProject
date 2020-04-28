import rospy
from occupancy_grid_python import OccupancyGridManager


class PathPlanning():
    def __init__(self):
        # initiliaze
        rospy.init_node('path_planning')


        # Subscribe to the nav_msgs/OccupancyGrid topic
        ogm = OccupancyGridManager('/move_base/global_costmap/costmap',
                            subscribe_to_updates=True)  # default False

               
if __name__ == '__main__':
    try:
        PathPlanning()
    except:
        rospy.loginfo("path_planning node terminated")

