#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32, Float32MultiArray
from geometry_msgs.msg import Twist
import argparse

def ui(message_type):
    # Initialize the ROS node
    rospy.init_node('ui_publisher', anonymous=False)

    # Create a publisher object based on the message type
    if message_type == 1:
        pub = rospy.Publisher('/ui_topic', String, queue_size=10)
    elif message_type == 2:
        pub = rospy.Publisher('/ui_topic', Float32, queue_size=10)
    elif message_type == 3:
        pub = rospy.Publisher('/ui_topic', Twist, queue_size=10)
    elif message_type == 4:
        pub = rospy.Publisher('/ui_topic', Float32MultiArray, queue_size=10)
    else:
        rospy.logerr("Unsupported message type ID: %d", message_type)
        return

    # Set the rate at which to publish messages (1 Hz)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        if message_type == 1:
            msg = String()
            msg.data = "Hello, ROS!"  # Example string data
        elif message_type == 2:
            msg = Float32()
            msg.data = 3.14  # Example float32 data
        elif message_type == 3:
            msg = Twist()
            msg.linear.x = 1.0  # Example linear velocity
            msg.angular.z = 0.5  # Example angular velocity
        elif message_type == 4:
            msg = Float32MultiArray()
            msg.data = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]  # Example data
        # Log the message to the console
        rospy.loginfo("Publishing: %s", msg)

        # Publish the message
        pub.publish(msg)

        # Sleep to maintain the publishing rate
        rate.sleep()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='ROS Node with command-line arguments.')
    parser.add_argument('--id', type=int, default=1, help='ID to specify the message type: 1 for String, 2 for Float32, 3 for Twist, 4 for float32multiarray')
    args = parser.parse_args()

    try:
        ui(args.id)
    except rospy.ROSInterruptException:
        pass
