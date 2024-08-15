#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import subprocess

class TerminalOutputProcessor:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('terminal_output_processor', anonymous=False)
        
        # Create a publisher object
        self.pub = rospy.Publisher('/terminal_output', String, queue_size=10)
        
        # Create a subscriber object
        self.sub = rospy.Subscriber('/command_input', String, self.command_callback)
        
        rospy.loginfo("Node initialized and waiting for commands on /command_input.")

    def command_callback(self, msg):
        command = msg.data
        rospy.loginfo("Received command: %s", command)
        self.capture_output(command)

    def capture_output(self, command):
        try:
            # Execute the command and capture the output
            result = subprocess.run(command, shell=True, check=True, capture_output=True, text=True)
            
            # Capture standard output and error output
            output = result.stdout.strip()
            error = result.stderr.strip()
            
            # Log the captured output and error
            if output:
                rospy.loginfo("Command Output: %s", output)
                self.publish_output(output)
            if error:
                rospy.logerr("Command Error: %s", error)
                self.publish_output(error)
        except subprocess.CalledProcessError as e:
            rospy.logerr("Command failed with error: %s", str(e))
            self.publish_output(str(e))

    def publish_output(self, output):
        # Create and publish a ROS message
        msg = String()
        msg.data = output
        rospy.loginfo("Publishing message: %s", msg.data)
        self.pub.publish(msg)

if __name__ == "__main__":
    try:
        node = TerminalOutputProcessor()
        rospy.spin() 
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception caught")

