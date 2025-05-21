# !/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header

import cv2

class DetermineColor(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.callback, 10)
        self.color_pub = self.create_publisher(Header, '/rotate_cmd', 10)
        self.bridge = CvBridge()
        self.count = 0

    def callback(self, data):
        try:
            # listen image topic
            img = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # prepare rotate_cmd msg
            # DO NOT DELETE THE BELOW THREE LINES!
            msg = Header()
            msg = data.header
            msg.frame_id = '0'  # default: STOP
            
            cv2.imshow('Image', img)
            cv2.waitKey(1)
    
            # determine background color
            # determine the color and assing +1, 0, or, -1 for frame_id
            # msg.frame_id = '+1' # CCW 
            # msg.frame_id = '0'  # STOP
            # msg.frame_id = '-1' # CW 
            black_lower  = np.array([0, 0, 0])
            black_upper  = np.array([180, 255, 80])

            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            mask_black = cv2.inRange(hsv, black_lower, black_upper)
            mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_CLOSE, kernel, iterations=2)

            contours, hier = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                msg.frame_id = '2'
            c = max(contours, key=cv2.contourArea)
            roi_mask = np.zeros_like(mask_black)
            cv2.drawContours(roi_mask, [c], -1, 255, thickness=-1)

            roi_mask = cv2.erode(roi_mask, kernel, iterations=1)
            #cv2.drawContours(img, [c], -1, (0, 0, 255), thickness=-1)
            cv2.imshow('Image', img)
            cv2.waitKey(1)

            mask_red = cv2.inRange(hsv, (0, 50, 50), (20, 255, 255)) | cv2.inRange(hsv, (160, 50, 50), (180, 255, 255))
            mask_green = cv2.inRange(hsv, (40, 50, 50), (80, 255, 255))
            mask_blue = cv2.inRange(hsv, (100, 50, 50), (140, 255, 255))

            red_in_roi   = cv2.bitwise_and(mask_red,   roi_mask)
            green_in_roi = cv2.bitwise_and(mask_green, roi_mask)
            blue_in_roi  = cv2.bitwise_and(mask_blue,  roi_mask)

            r_count = cv2.countNonZero(red_in_roi)
            g_count = cv2.countNonZero(green_in_roi)
            b_count = cv2.countNonZero(blue_in_roi)

            if r_count > g_count and r_count > b_count:
                msg.frame_id = '-1'
            elif b_count > r_count and b_count > g_count:
                msg.frame_id = '+1'
            else:
                msg.frame_id = '0'
            # msg.frame_id = str(r_count)+'.'+str(b_count)+'.'+str(g_count)
            print(msg.frame_id)
            
            self.color_pub.publish(msg)
            
            
            
        except CvBridgeError as e:
            self.get_logger().error('Failed to convert image: %s' % e)


if __name__ == '__main__':
    rclpy.init()
    detector = DetermineColor()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()
