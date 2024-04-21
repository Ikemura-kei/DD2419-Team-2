from rclpy.node import Node
import rclpy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from cv_bridge import CvBridge

class ArmObjectDetection(Node):
    def __init__(self):
        super().__init__('arm_object_detection')
        
        self.bridge = CvBridge()
        self.joycon_sub = self.create_subscription(Image, topic='/image_rect', callback=self.rect_img_cb, qos_profile=10)

    def rect_img_cb(self, img:Image):
        print("Here")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
        
def main():
    print('Hi from arm_object_detection.')
    
    rclpy.init()
    
    arm_object_detection = ArmObjectDetection()
    
    rclpy.spin(arm_object_detection)
    
    arm_object_detection.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()