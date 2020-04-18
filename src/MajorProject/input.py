import rospy

class Input():
    def __init__(self):
        # initiliaze
        rospy.init_node('input')
        
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            print("Please choose a destination from the following choices:\n")
            print("A: Atrium\nB: Computer Lab\nC: Electronics Lab\nD: Team Room\nE:Main Office\n")
            r.sleep()

if __name__ == '__main__':
    try:
        Input()
    except:
        rospy.loginfo("Input node terminated.")
