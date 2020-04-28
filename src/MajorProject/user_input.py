import rospy
from geometry_msgs.msg import Point

class UserInput():
    def __init__(self):
        rospy.init_node('user_input')
        self.pub = rospy.Publisher('user_input', Point, queue_size=10)
        #self.sub = rospy.Subscriber('nav_result', Point, self.callback)

        quit = False
        while (not rospy.is_shutdown()) and quit is False:
            print("Please choose from the following choices:")
            choice = raw_input("A: Atrium\nB: Computer Lab\nC: Electronics Lab\nD: Team Room\nE: Main Office\nF: Quit\n")

            # Highlight locations on map of DEH
            atrium_pos = {'name': 'Atrium', 'x': 2, 'y' : -5}
            comp_lab_pos = {'name': 'Computer Lab', 'x': 8, 'y' : -1}
            elec_lab_pos = {'name': 'Electronics Lab', 'x': 1, 'y' : -1}
            team_room_pos = {'name': 'Team Room', 'x': -8, 'y' : -6}
            main_office_pos =  {'name': 'Main Office', 'x': -8, 'y' : 3}
            pos = Point()

            valid = True
            if choice == 'A':
                pos = Point(atrium_pos['x'], atrium_pos['y'], 0.000)
            elif choice == 'B':
                pos = Point(comp_lab_pos['x'], comp_lab_pos['y'], 0.000)
            elif choice == 'C':
                pos = Point(elec_lab_pos['x'], elec_lab_pos['y'], 0.000)
            elif choice == 'D':
                pos = Point(team_room_pos['x'], team_room_pos['y'], 0.000)
            elif choice == 'E':
                pos = Point(main_office_pos['x'], main_office_pos['y'], 0.000)
            elif choice == 'F':
                quit = True
                valid = False
                rospy.loginfo("user_input node terminated")
            else:
                print('Invalid choice\n')
                valid = False

            # Publish point if valid
            if valid:
                self.pub.publish(pos)
     

    def callback(self, data):
        if data is True:
            print("We've reached the destination.")
        else:
            print("Sorry, I've failed to bring you to the correct destination.")


if __name__ == '__main__':
    try:
        UserInput()
    except rospy.ROSInterruptException:
        rospy.loginfo("user_input node terminated")
