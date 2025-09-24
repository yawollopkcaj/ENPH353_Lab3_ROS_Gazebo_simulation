#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LineFollower:
    """
    @brief Line following robot controller using vision and PID control.
    """
    def __init__(self):
        """
        @brief Initialize the LineFollower node, parameters, and ROS I/O.
        """
        rospy.init_node('line_follower', anonymous=True)

        # Parameters
        self.image_topic = rospy.get_param('~image_topic', '/robot/camera1/image_raw')
        self.cmd_topic = rospy.get_param('~cmd_topic', '/cmd_vel')
        self.hz = rospy.get_param('~rate_hz', 30)
        self.v_forward = rospy.get_param('~v_forward', 0.25) # m/s
        
        # PID parameters
        self.kp = rospy.get_param('~kp', 0.004)
        self.ki = rospy.get_param('~ki', 0.0)
        self.kd = rospy.get_param('~kd', 0.0)

        # Vision parameters
        self.roi_height_frac = np.clip(rospy.get_param('~roi_height_frac', 0.35), 0.05, 0.95)
        self.binary_thresh = rospy.get_param('~binary_thresh', 110)
        self.erode_iters = int(rospy.get_param('~erode_iters', 1))
        self.dilate_iters = int(rospy.get_param('~dilate_iters', 2))

        # ROS I/O
        self.bridge = CvBridge()
        self.pub_cmd = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.sub_img = rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)

        # Integrator state
        self.err_int = 0.0
        self.prev_err_x = 0.0

        rospy.spin()

    def image_cb(self, msg):
        """
        @brief Callback for processing incoming camera images.
        @param msg The incoming image message.
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            return
        
        h, w, _ = frame.shape
        roi_h = int(self.roi_height_frac * h)
        roi = frame[h - roi_h : h, 0 : w]   # bottom strip

        # Preprocess grayscale -> blur -> binary
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5,5), 0)
        _, bw = cv2.threshold(gray, self.binary_thresh, 255, cv2.THRESH_BINARY_INV)

        # Image clean up
        if self.erode_iters > 0:
            bw = cv2.erode(bw, None, iterations=self.erode_iters)
        if self.dilate_iters > 0:
            bw = cv2.dilate(bw, None, iterations=self.dilate_iters)

        # Compute centroid in Region of Interest (ROI)
        M = cv2.moments(bw)
        has_line = M["m00"] > 0

        # Default: slow forward, no turning
        twist = Twist()
        twist.linear.x = self.v_forward

        if has_line:
            cx = int(M["m10"] / M["m00"]) # x of centroid in ROI
            err_x = (cx - w / 2.0) # pixels; +right, -left
            self.err_int += err_x / float(self.hz) # integral

            # Derivative term
            err_deriv = (err_x - self.prev_err_x) * self.hz
            self.prev_err_x = err_x

            # Steering command (prop + int + deriv)
            steer = -(self.kp * err_x + self.ki * self.err_int + self.kd * err_deriv)

            twist.angular.z = float(steer)
        else:
            # Lost line: slow down and spin gently to reacquire
            twist.linear.x  = 0.2 # m/s
            twist.angular.z = 2.0 # rad/s

        self.pub_cmd.publish(twist)
    
if __name__ == "__main__":
    LineFollower()
