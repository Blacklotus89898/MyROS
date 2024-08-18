#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import argparse

class VideoCamNode:
    def __init__(self):
        parser = argparse.ArgumentParser(description='ROS Node with command-line arguments.')
        parser.add_argument('--id', type=int, default=0, help='ID to use for this node')
        args = parser.parse_args()
        id = args.id
        rospy.init_node('cam.py'+ str(id), anonymous=False)

        self.image_pub = rospy.Publisher('/camera_frames_'+str(id), Image, queue_size=10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(id)

        if not self.cap.isOpened():
            rospy.logerr("Error: Could not open video capture.")
            rospy.signal_shutdown("Could not open video capture")

    def run(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logerr("Error: Could not read frame.")
                continue

            frame_resized = cv2.resize(frame, (640, 480))  # Ensure this matches the frontend expectation
            rgb_frame = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)

            try:
                # Convert the frame to ROS Image message with encoding "rgb8"
                ros_image = self.bridge.cv2_to_imgmsg(rgb_frame, encoding="rgb8")
                rospy.loginfo(f"Publishing image with width: {ros_image.width}, height: {ros_image.height}, data length: {len(ros_image.data)}")
                self.image_pub.publish(ros_image)
            except CvBridgeError as e:
                rospy.logerr("CvBridgeError: %s" % str(e))

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
