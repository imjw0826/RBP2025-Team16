# !/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header

import cv2

class DetermineColor(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.image_sub = self.create_subscription(Image, '/color', self.callback, 10)
        self.color_pub = self.create_publisher(Header, '/rotate_cmd', 10)
        self.bridge = CvBridge()
        self.count = 0

    def callback(self, data):
        try:
            # listen image topic
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # prepare rotate_cmd msg
            # DO NOT DELETE THE BELOW THREE LINES!
            msg = Header()
            msg = data.header
            msg.frame_id = '0'  # default: STOP
    
            # determine background color
            # TODO 
            # determine the color and assing +1, 0, or, -1 for frame_id
            # msg.frame_id = '+1' # CCW 
            # msg.frame_id = '0'  # STOP
            # msg.frame_id = '-1' # CW 
            
            # self.count+=1
            #i f self.count > 30 and self.count < 60:
            #     msg.frame_id = '+1'
            # elif self.count > 60 and self.count < 90:
            #     msg.frame_id = '-1'
            # elif self.count > 90 and self.count < 120:
            #     msg.frame_id = '0'
            # elif self.count > 120 and self.count < 150:
            #     msg.frame_id = '+1'
            # elif self.count > 150 and self.count < 180:
            #     msg.frame_id = '-1'
            # elif self.count > 180 and self.count < 210:
            #     msg.frame_id = '0'
            
            
            
            cv2.imshow('Image', image)
            cv2.waitKey(1)
            
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            blue = image[20, 10, 0]
            green = image[20, 10, 1]
            red = image[20, 10, 2]
            
            if blue > green and blue > red :
            	msg.frame_id = '+1'
            elif green > blue and green > red :
            	msg.frame_id = '0'
            elif red > green and red > blue :
            	msg.frame_id = '-1'
            # publish color_state
            self.color_pub.publish(msg)
            
            
            
        except CvBridgeError as e:
            self.get_logger().error('Failed to convert image: %s' % e)


if __name__ == '__main__':
    rclpy.init()
    detector = DetermineColor()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

