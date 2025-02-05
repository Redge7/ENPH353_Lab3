#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LineFollower:
    def __init__(self):
        rospy.init_node('line_follower', anonymous=True)
        self.bridge = CvBridge()

        # Subscribe to the robot's camera image topic
        self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)

        # Publisher for velocity commands
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()

    def image_callback(self, msg):
        """Processes the camera image and generates velocity commands."""
        try:
            # Convert ROS image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Convert to grayscale and apply thresholding
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 130, 255, cv2.THRESH_BINARY_INV)

            # Crop to focus on the bottom part of the image (where the line is)
            height, width = binary.shape
            roi = binary[int(height * 0.5):, :]

            # Find contours of the line
            M = cv2.moments(roi)
            if M["m00"] > 0:
                # Computing centroid (cx) of the detected line
                cx = int(M["m10"] / M["m00"])

                # Calculate error (deviation from center)
                error = cx - (width // 2)

                #Determining how much to steer left or right
                Kp = 0.02
                angular_z = -Kp * error

                # Set linear and angular velocity
                self.twist.linear.x = 4.0  # Move forward at constant speed
                self.twist.angular.z = angular_z  # Adjust steering
            else:
                # Stop if no line is detected and keep rotating till line is detected again
                self.twist.linear.x = 0.0
                self.twist.angular.z = 4.0

            # Publish the velocity command
            self.cmd_pub.publish(self.twist)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def run(self):
        rospy.spin()

#If this scriipt is called, create a LineFollower object and execute, can be used as a class definition otherwise
if __name__ == '__main__':
    follower = LineFollower()
    follower.run()
