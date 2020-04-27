import rospy
import string
from std_msgs.msg import String


class Input():
    def __init__(self):
        self.pub = rospy.Publisher('user_input', String, queue_size=10)
        rospy.init_node('user_input')
        
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            print("Please choose a destination from the following choices:\n")
            choice = raw_input("A: Atrium\nB: Computer Lab\nC: Electronics Lab\nD: Team Room\nE: Main Office\n")
            print(choice)
            '''
            # Highlight locations on map of DEH
            atrium_pos = {'x': 1.57, 'y' : -6.00}
            comp_lab_pos = {'x': 10.3, 'y' : -1.93}
            elec_lab_pos = {'x': -3.52, 'y' : -2.23}
            team_room_pos = {'x': -7.01, 'y' : -4.44}
            main_office_pos =  {'x': -5.87, 'y' : 1.33}
            pos = {}
            valid = True
            if choice == 'A':
                pos = atrium_pos
            elif choice == 'B':
                pos = comp_lab_pos
            elif choice == 'C':
                pos = elec_lab_pos
            elif choice == 'D':
                pos = team_room_pos
            elif choice == 'E':
                pos = main_office_pos
            else:
                print('Invalid choice\n')
                valid = False
            self.pub.publish(choice)
            r.sleep()
            '''

if __name__ == '__main__':
    try:
        Input()
    except rospy.ROSInterruptException:
        rospy.loginfo("Input node terminated.")
