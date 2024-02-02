#!/usr/bin/env python

from rclpy.node import Node
from sensor_msgs.msg import Image
import rclpy
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        self.bridge = CvBridge()

        self.img_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)

        self.img_sub

    def image_callback(self, msg:Image):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        print(cv_image.shape)
        
        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            cv2.circle(cv_image, (50,50), 10, 255)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        # print('=> Image received')

def main():
    print('Hi from object_detection.')

    rclpy.init()

    object_detection_node = ObjectDetectionNode()

    rclpy.spin(object_detection_node)

    object_detection_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
