#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class VideoCamNode:
    def __init__(self):
        rospy.init_node('cam.py', anonymous=True)

        # Publisher to publish video frames
        self.image_pub = rospy.Publisher('/cam.py', Image, queue_size=10)

        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Open the video capture
        self.cap = cv2.VideoCapture(5)

        # Check if the video capture opened successfully
        if not self.cap.isOpened():
            rospy.logerr("Error: Could not open video capture.")
            rospy.signal_shutdown("Could not open video capture")

    def run(self):
        while not rospy.is_shutdown():
            # Capture frame-by-frame
            ret, frame = self.cap.read()
            if not ret:
                rospy.logerr("Error: Could not read frame.")
                continue

            # Convert OpenCV image to ROS image message
            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_pub.publish(ros_image)
            except CvBridgeError as e:
                rospy.logerr("CvBridgeError: %s" % str(e))

            # Display the frame
            cv2.imshow('Video Feed', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        node = VideoCamNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

