#!/usr/bin/env python3
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class LaneDetector:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply Canny edge detection
        edges = cv2.Canny(blur, 50, 150)

        # Create roi mask
        mask = np.zeros_like(edges)
        height, width = edges.shape
        vertices = np.array([[(0, height), (width/2, height/2), (width, height)]], dtype=np.int32)
        cv2.fillPoly(mask, vertices, 255)
        masked_edges = cv2.bitwise_and(edges, mask)

        # Apply Hough line detection
        lines = cv2.HoughLinesP(masked_edges, rho=2, theta=np.pi/180, threshold=50, minLineLength=100, maxLineGap=200)

        # Draw the lines on the original image
        line_image = np.zeros_like(cv_image)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 3)

        # Combine the line image with the original image
        result = cv2.addWeighted(cv_image, 0.8, line_image, 1, 0)

        # Calculate the steering angle
        angle = 0.0
        if lines is not None:
            left_lines = []
            right_lines = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x2 == x1:
                    continue  # ignore vertical lines
                slope = (y2 - y1) / (x2 - x1)
                if slope < 0:
                    left_lines.append((slope, x1, y1, x2, y2))
                else:
                    right_lines.append((slope, x1, y1, x2, y2))
            left_lines = np.array(left_lines)
            right_lines = np.array(right_lines)
            if len(left_lines) > 0:
                left_avg = np.average(left_lines, axis=0)
                slope, x1, y1, x2, y2 = left_avg
                angle = np.arctan(slope) * 180 / np.pi
            elif len(right_lines) > 0:
                right_avg = np.average(right_lines, axis=0)
                slope, x1, y1, x2, y2 = right_avg
                angle = np.arctan(slope)*180 / np.pi

        # Publish the steering command
        twist = Twist()
        twist.linear.x = 1.5  # constant forward speed
        twist.angular.z = angle / 50  # scaling factor for turning
        self.cmd_vel_pub.publish(twist)

        # Display the result
        cv2.imshow("Result", result)
        cv2.waitKey(1)

def main(args):
    rospy.init_node('lane_detector', anonymous=False)
    lane_detector = LaneDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down...")
    cv2.destroyAllWindows()

if __name__=="__main__":
     main(sys.argv)
