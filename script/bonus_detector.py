#!/usr/bin/env python3
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class BonusBallTracker:
    def __init__(self):
        rospy.init_node('bonus_vision_depth_only')

        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)

        self.depth_topic     = rospy.get_param("~depth_topic",      "/camera/depth/image_raw")
        self.min_depth       = rospy.get_param("~min_depth",        0.30)
        self.max_depth       = rospy.get_param("~max_depth",        3.50)
        self.depth_band      = rospy.get_param("~depth_band",       0.35)
        self.min_area        = rospy.get_param("~min_area",          200)
        self.min_radius      = rospy.get_param("~min_radius",          8)
        self.max_radius      = rospy.get_param("~max_radius",        180)
        self.min_circularity = rospy.get_param("~min_circularity",  0.45)
        self.min_fill_ratio  = rospy.get_param("~min_fill_ratio",   0.40)
        self.target_distance = rospy.get_param("~target_distance",  0.80)
        self.stop_distance   = rospy.get_param("~stop_distance",    0.45)
        self.max_linear      = rospy.get_param("~max_linear",       0.35)
        self.linear_gain     = rospy.get_param("~linear_gain",      0.45)
        self.angular_gain    = rospy.get_param("~angular_gain",     0.70)
        self.memory_timeout  = rospy.get_param("~memory_timeout",   2.0)

        self.edge_reject_frac = rospy.get_param("~edge_reject_frac", 0.72)
        self.cmd_alpha = rospy.get_param("~cmd_alpha", 0.35)

        self.hq_circularity = rospy.get_param("~hq_circularity", 0.60)
        self.hq_fill_ratio  = rospy.get_param("~hq_fill_ratio",  0.55)

        self.image_width = 640

        self.last_center    = None
        self.last_distance  = None
        self.last_seen_time = rospy.Time(0)

        self.smooth_linear  = 0.0
        self.smooth_angular = 0.0

        rospy.Subscriber(
            self.depth_topic, Image, self.depth_callback,
            queue_size=1, buff_size=2**24
        )

    def _to_meters(self, depth_image):
        if depth_image.dtype == np.uint16:
            return depth_image.astype(np.float32) / 1000.0
        return depth_image.astype(np.float32)

    def _recent_track_exists(self):
        if self.last_distance is None or self.last_center is None:
            return False
        return (rospy.Time.now() - self.last_seen_time).to_sec() < self.memory_timeout

    def _build_depth_mask(self, depth_m):
        valid = depth_m[(depth_m > self.min_depth) & (depth_m < self.max_depth)]
        if valid.size < 50:
            return None, None

        kernel = np.ones((5, 5), np.uint8)

        if self._recent_track_exists():
            ref_depth = float(self.last_distance)
            lower = max(self.min_depth, ref_depth - 0.10)
            upper = min(self.max_depth, ref_depth + self.depth_band)
            mask  = ((depth_m >= lower) & (depth_m <= upper)).astype(np.uint8) * 255
            mask  = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
            mask  = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
            return mask, ref_depth

        else:
            best_mask       = None
            best_ref        = None
            best_ball_score = -1e9
            step            = self.depth_band * 0.5 

            d = self.min_depth
            while d < self.max_depth:
                lower     = d
                upper     = min(self.max_depth, d + self.depth_band)
                candidate = ((depth_m >= lower) & (depth_m <= upper)).astype(np.uint8) * 255
                candidate = cv2.morphologyEx(candidate, cv2.MORPH_OPEN,  kernel, iterations=1)
                candidate = cv2.morphologyEx(candidate, cv2.MORPH_CLOSE, kernel, iterations=2)

                ball = self._find_best_ball(depth_m, candidate)
                if ball is not None and ball["score"] > best_ball_score:
                    best_ball_score = ball["score"]
                    best_mask       = candidate
                    best_ref        = (lower + upper) / 2.0
                d += step

            return best_mask, best_ref

    def _find_best_ball(self, depth_m, mask):
        contours_info = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]

        best       = None
        best_score = -1e9

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter <= 0:
                continue

            circularity = 4.0 * np.pi * area / (perimeter * perimeter)
            (x, y), radius = cv2.minEnclosingCircle(cnt)

            if radius < self.min_radius or radius > self.max_radius:
                continue

            fill_ratio = area / (np.pi * radius * radius + 1e-6)

            if circularity < self.min_circularity:
                continue
            if fill_ratio < self.min_fill_ratio:
                continue

            half_w     = self.image_width / 2.0
            edge_limit = self.edge_reject_frac * half_w
            if abs(x - half_w) > edge_limit:
                continue

            cnt_mask = np.zeros(mask.shape, dtype=np.uint8)
            cv2.drawContours(cnt_mask, [cnt], -1, 255, thickness=-1)

            region = depth_m[
                (cnt_mask > 0) &
                (depth_m > self.min_depth) &
                (depth_m < self.max_depth)
            ]
            if region.size < 30:
                continue

            distance = float(np.median(region))

            center_penalty = 0.0
            depth_penalty  = 0.0
            if self._recent_track_exists():
                dx = x - self.last_center[0]
                dy = y - self.last_center[1]
                center_penalty = 0.003 * np.hypot(dx, dy)
                depth_penalty  = 1.5   * abs(distance - self.last_distance)

            score = (
                2.0  * circularity +
                1.0  * fill_ratio  +
                0.002 * area       -
                0.15  * distance   -
                center_penalty     -
                depth_penalty
            )

            if score > best_score:
                best_score = score
                best = {
                    "cx": int(x), "cy": int(y), "radius": int(radius),
                    "distance": distance, "contour": cnt,
                    "circularity": circularity, "fill_ratio": fill_ratio,
                    "score": score
                }

        return best

    def _make_debug_image(self, depth_m, mask=None):
        clipped = np.clip(depth_m, self.min_depth, self.max_depth)
        norm    = ((clipped - self.min_depth) /
                   (self.max_depth - self.min_depth + 1e-6) * 255.0).astype(np.uint8)
        norm[~np.isfinite(depth_m)] = 0
        norm[depth_m <= 0]          = 0
        debug = cv2.cvtColor(norm, cv2.COLOR_GRAY2BGR)
        if mask is not None:
            overlay = debug.copy()
            overlay[mask > 0] = (0, 180, 0)
            debug = cv2.addWeighted(debug, 0.75, overlay, 0.25, 0)
        return debug

    def _smooth_cmd(self, target_linear, target_angular):
        """Applies exponential moving average to smooth commands and prevent twitching."""
        self.smooth_linear  = (self.cmd_alpha * target_linear  +
                               (1.0 - self.cmd_alpha) * self.smooth_linear)
        self.smooth_angular = (self.cmd_alpha * target_angular +
                               (1.0 - self.cmd_alpha) * self.smooth_angular)
        return self.smooth_linear, self.smooth_angular

    def depth_callback(self, data):
        move = Twist()

        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr_throttle(2, f"CvBridge error: {e}")
            self.pub.publish(move)
            return

        depth_m = self._to_meters(depth_image)
        depth_m = np.array(depth_m, dtype=np.float32)

        if depth_m.ndim != 2:
            rospy.logwarn_throttle(2, "Depth image is not single-channel.")
            self.pub.publish(move)
            return

        self.image_width               = depth_m.shape[1]
        depth_m[~np.isfinite(depth_m)] = 0.0
        depth_m = cv2.GaussianBlur(depth_m, (5, 5), 0)

        mask, ref_depth = self._build_depth_mask(depth_m)
        debug_image     = self._make_debug_image(depth_m, mask)

        if mask is None:
            lin, ang = self._smooth_cmd(0.0, 0.0)
            move.linear.x  = lin
            move.angular.z = ang
            self.pub.publish(move)
            cv2.imshow("Depth Ball Tracker", debug_image)
            cv2.waitKey(3)
            rospy.loginfo_throttle(2, "No valid depth data.")
            return

        ball = self._find_best_ball(depth_m, mask)

        if ball is not None:
            cx       = ball["cx"]
            cy       = ball["cy"]
            radius   = ball["radius"]
            distance = ball["distance"]
            circ     = ball["circularity"]
            fill     = ball["fill_ratio"]

            HIGH_QUALITY = (circ >= self.hq_circularity and fill >= self.hq_fill_ratio)

            if HIGH_QUALITY:
                err_x       = cx - (self.image_width / 2.0)
                raw_angular = -self.angular_gain * (err_x / (self.image_width / 2.0))
            else:
                err_x       = cx - (self.image_width / 2.0) 
                raw_angular = 0.0

            if HIGH_QUALITY and distance > self.target_distance:
                raw_linear = min(self.max_linear,
                                 self.linear_gain * (distance - self.target_distance))
            else:
                raw_linear = 0.0

            if distance <= self.stop_distance:
                raw_linear = 0.0

            lin, ang = self._smooth_cmd(raw_linear, raw_angular)
            move.linear.x  = lin
            move.angular.z = ang

            self.last_center    = (cx, cy)
            self.last_distance  = distance
            self.last_seen_time = rospy.Time.now()

            cv2.drawContours(debug_image, [ball["contour"]], -1, (0, 255, 255), 2)
            cv2.circle(debug_image, (cx, cy), radius, (0, 255, 0), 2)
            cv2.circle(debug_image, (cx, cy), 3,      (0, 0, 255), -1)

            quality_tag = "HQ" if HIGH_QUALITY else "LQ"
            mode        = "LOCKED" if self._recent_track_exists() else "SWEEP"
            rospy.loginfo_throttle(
                1.0,
                f"[{mode}][{quality_tag}] dist={distance:.2f}m err_x={err_x:.0f} "
                f"circ={circ:.2f} fill={fill:.2f} "
                f"lin={lin:.3f} ang={ang:.3f}"
            )

        else:
            lin, ang = self._smooth_cmd(0.0, 0.0)
            move.linear.x  = lin
            move.angular.z = ang
            rospy.loginfo_throttle(1.0, "No circular depth blob found.")

        self.pub.publish(move)
        cv2.imshow("Depth Ball Tracker", debug_image)
        cv2.waitKey(3)

if __name__ == '__main__':
    tracker = BonusBallTracker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()