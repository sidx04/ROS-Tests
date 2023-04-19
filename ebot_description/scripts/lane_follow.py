#!/usr/bin/env python

import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from time import sleep

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


VIDEO_DIMENSIONS = (640, 480)

    
class Lane():
    def __init__(self) -> None:
        rospy.init_node('lane_follow', anonymous=True)
        
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        rospy.Subscriber('/camera/color/image_raw', Image, self.drive)
        rospy.spin()

 
    def process_frame(self):
        img = cv.GaussianBlur(self.raw_frame, (3,3), 100)
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        
        hsv_low = np.array([123, 244, 0], np.uint8)
        hsv_high = np.array([179, 255, 160], np.uint8)
        mask = cv.inRange(hsv, hsv_low, hsv_high)
        
        res = cv.bitwise_and(hsv, hsv, mask=mask)
        
        h = res.shape[0]
        histogram = np.sum(res, axis=0)
        mid = np.int(histogram.shape[0]/2)
        print('mid',mid)
        l_max = np.argmax(histogram[:mid])
        r_max = np.argmax(histogram[mid:]) + mid
        print(l_max,r_max)
        center = np.int((l_max+r_max)/2)
        print(center)

        roi = np.float32(
            ((200, 92),
                (50, 260),
                (610, 260),
                (440, 92))
            )

        pad = 80
        desired = np.float32(
            ((pad, 0),
                (pad, 480),
                (640-pad, 480),
                (640-pad, 0))
            )

        transformation_matrix = cv.getPerspectiveTransform(roi, desired)

        warped_image = cv.warpPerspective(res, transformation_matrix, VIDEO_DIMENSIONS, flags=(cv.INTER_LINEAR))

        self.processed_frame = warped_image
        

    def find_center(self, plot=False):
        image = self.processed_frame
        h = image.shape[0]
        histogram = np.sum(image[h//2:, :], axis=0)
        
        mid = np.int(histogram.shape[0]/2)
        
        l_max = np.argmax(histogram[:mid-50])
        r_max = np.argmax(histogram[mid+50:]) + mid
        
        self.center = np.int((l_max+r_max)/2)
        
        if plot:
            figure, (ax1, ax2) = plt.subplots(2,1)
            figure.set_size_inches(10, 5)
            ax1.imshow(image, cmap='gray')
            ax1.set_title("Warped Binary Frame")
            ax2.plot(histogram)
            ax2.set_title("Histogram Peaks")
            plt.show()
            sleep(0.1)
            plt.close()


    def drive(self, image_raw):
        bridge = CvBridge()
        self.raw_frame = bridge.imgmsg_to_cv2(image_raw)
        # cv.imwrite('test.jpg', self.raw_frame)
        
        self.process_frame()
        self.find_center()
        
        true_center = 320
        print(f'true center: {true_center}\tcenter: {self.center}')
        
        scale_factor = -0.025
        angular = scale_factor * (self.center - true_center)

        # if abs(angular) < 0.75:
        #     angular = 0
        
        command = Twist()
        command.linear.x = 2.5
        command.angular.z = angular
        
        self.pub.publish(command)
        print(f'Publishing {command}')


if __name__ == '__main__':
    try:
        Lane()
    except rospy.ROSInterruptException:
        print('Exiting..')