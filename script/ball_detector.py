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
        self.pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)

        # HSV color range for green ball
        # If detection is unreliable, these are the only values to tune
        self.lower_green = np.array([35, 40, 40])
        self.upper_green = np.array([90, 255, 255])

        # Distance control — real metres from depth sensor
        self.TARGET_DIST = 1.00   # follow at 1 metre (~3 feet)
        self.STOP_DIST   = 0.80   # reverse if closer than this
        self.MAX_SPEED   = 0.20   # maximum forward AND reverse speed

        # Store latest depth frame
        self.latest_depth = None

        # RGB subscriber — finds the ball position
        rospy.Subscriber("/camera/color/image_raw", Image,
                         self.rgb_callback,
                         queue_size=1, buff_size=2**24)

        # Depth subscriber — measures real distance to ball
        rospy.Subscriber("/camera/depth/image_raw", Image,
                         self.depth_callback,
                         queue_size=1, buff_size=2**24)

    # ------------------------------------------------------------------
    def depth_callback(self, data):
        """Store the latest depth frame for use in rgb_callback."""
        try:
            depth = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr_throttle(2, f"Depth CvBridge error: {e}")
            return

        # Normalise mm → metres for 16UC1 cameras (RealSense, Kinect etc.)
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) / 1000.0

        self.latest_depth = depth

    # ------------------------------------------------------------------
    def rgb_callback(self, data):
        """Detect ball using color, measure distance using depth, control robot."""
        if self.latest_depth is None:
            return  # wait until we have at least one depth frame

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr_throttle(2, f"RGB CvBridge error: {e}")
            return

        # --- Color detection ---
        hsv  = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        move = Twist()

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area    = cv2.contourArea(largest)

            if area > 1500:
                # Ball centre in image pixels
                M  = cv2.moments(largest)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # Steering error — how far off-centre the ball is
                err = cx - (cv_image.shape[1] / 2)

                # --- Read real distance from depth at ball centre ---
                h, w = self.latest_depth.shape
                cx_s = max(0, min(cx, w - 1))
                cy_s = max(0, min(cy, h - 1))
                distance = float(self.latest_depth[cy_s, cx_s])

                # If centre pixel is invalid (NaN/zero), sample a patch around it
                if np.isnan(distance) or distance <= 0:
                    patch = self.latest_depth[max(0, cy_s-5):cy_s+5,
                                              max(0, cx_s-5):cx_s+5]
                    valid    = patch[patch > 0]
                    distance = float(np.median(valid)) if len(valid) > 0 else 999.0

                # --- Steering: always active when ball is visible ---
                move.angular.z = -float(err) / 250

                # --- Forward / reverse based on real distance ---
                if distance > self.TARGET_DIST:
                    # Ball is far — move forward, scale speed by how far
                    move.linear.x = min(self.MAX_SPEED,
                                        0.30 * (distance - self.TARGET_DIST))

                elif distance < self.STOP_DIST:
                    # Ball is too close — reverse proportionally
                    move.linear.x = max(-self.MAX_SPEED,
                                        -0.30 * (self.TARGET_DIST - distance))

                else:
                    # Ball is in the acceptable zone — hold position
                    move.linear.x = 0.0

                # Determine state label for logging
                if distance > self.TARGET_DIST:
                    state = "FORWARD"
                elif distance < self.STOP_DIST:
                    state = "REVERSE"
                else:
                    state = "HOLDING"

                rospy.loginfo_throttle(2,
                    f"[{state}] dist={distance:.2f}m | err={err:.0f} | "
                    f"lin={move.linear.x:.3f} | ang={move.angular.z:.3f}")

        self.pub.publish(move)


if __name__ == '__main__':
    tracker = BallTracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
