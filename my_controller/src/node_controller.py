import os
import rospy
from std_msgs.msg import String

def control_callback(msg):
    command = msg.data
    rospy.loginfo("Received command: %s", command)

    if command == "start":
        os.system('rosrun beginner_tutorial talker &')  # & to run in background
        #os.system('rosrun node_control simple_publisher.py &')  # & to run in background
        rospy.loginfo("Node started.")

    elif command == "stop":
        #os.system('pkill -f simple_publisher.py')  # Use pkill to kill the process
        os.system('rosnode kill talker')  # Use pkill to kill the process
        rospy.loginfo("Node stopped.")

    else:
        try:
            os.system(command)
        except:
            rospy.logwarn("Invalid command: %s", command)

def node_controller():
    rospy.init_node('node_controller')
    rospy.Subscriber('node_controller', String, control_callback)
    rospy.loginfo("Node controller is ready.")
    rospy.spin()

if __name__ == "__main__":
    try:
        node_controller()
    except rospy.ROSInterruptException:
        pass

