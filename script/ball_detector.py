#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

class BallTracker:
    def __init__(self):
        rospy.init_node('ball_vision')
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        
        self.lower_green = np.array([35, 40, 40])
        self.upper_green = np.array([90, 255, 255])

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 1500:
                M = cv2.moments(largest)
                cx = int(M['m10']/M['m00'])
                err = cx - (cv_image.shape[1]/2)
                
                move = Twist()
                move.linear.x = 0.15
                move.angular.z = -float(err) / 250
                self.pub.publish(move)
                rospy.loginfo_throttle(2, "Tracking Green Ball...")

if __name__ == '__main__':
    tracker = BallTracker()
    rospy.Subscriber("/camera/color/image_raw", Image, tracker.callback)
    rospy.spin()