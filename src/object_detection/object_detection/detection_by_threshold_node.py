import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
import cv2
import numpy as np
import cv_bridge

from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Bool

class ThresholdMethod:
    OTSU = 0
    FIXED = 1
    IN_RANGE = 2
    
    def __init__(self, method):
        self.method = method
        
def pre_proc(image, method:ThresholdMethod):
    if method == ThresholdMethod.OTSU or method == ThresholdMethod.FIXED:
        image = cv2.cvtColor(image, cv2.COLOR_YUV2GRAY_YUY2)
    else:
        image = cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUY2)
    
    H, W, _ = image.shape
    image = cv2.resize(image, (int(W/1.75), int(H/1.75)))
    image = cv2.GaussianBlur(image, (5, 5), 0)
    return image

class DetectionByThresholdNode(Node):
    def __init__(self):
        super().__init__('detection_by_threshold_node')
        
        self.enable = False
        
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.enable_sub = self.create_subscription(
            Bool,
            '/detection_by_threshold_node_enable',
            self.enable_cb,
            10)
        self.enable_sub  # prevent unused variable warning
        
        self.bbox_pub = self.create_publisher(Int16MultiArray, '/theshold_bbox', 10)
        
        self.bridge = cv_bridge.CvBridge()
        self.THRESHOLD_METHOD = ThresholdMethod.IN_RANGE
        
        self.MORPH_KERNEL = np.ones((5,5),np.uint8)
        
    def enable_cb(self, msg:Bool):
        self.enable = msg.data

    def image_callback(self, msg:Image):
        self.get_logger().info('Received an image')
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='yuv422_yuy2')
        cv_image = pre_proc(cv_image, self.THRESHOLD_METHOD)
        
        if not self.enable:
            cv2.imshow('image', cv_image)
            cv2.waitKey(1)
            return
        
        if self.THRESHOLD_METHOD == ThresholdMethod.FIXED:
            ret, result = cv2.threshold(cv_image,196,255,cv2.THRESH_BINARY)
        elif self.THRESHOLD_METHOD == ThresholdMethod.OTSU:
            ret, result = cv2.threshold(cv_image,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        elif self.THRESHOLD_METHOD == ThresholdMethod.IN_RANGE:
            ground_1 = cv2.inRange(cv_image, (45, 45, 45), (255, 255, 255))
            # ground_2 = cv2.inRange(cv_image, (130, 130, 130), (220, 220, 220))
            
            diffs = np.sum(((cv_image.astype(np.float32) - np.mean(cv_image, axis=2, keepdims=True))**2), axis=2)
            cv2.imshow('diffs', (diffs / 50).astype(np.uint8))
            ground_3 = (diffs <= 500)
            result = np.where(((ground_1) * ground_3) >= 1, 255, 0).astype(np.uint8)

            result = cv2.morphologyEx(result, cv2.MORPH_CLOSE, self.MORPH_KERNEL)
        
        result = np.where(result == 255, 0, 255).astype(np.uint8)

        contours = cv2.findContours(result, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        bbox = []
        for cnt in contours:
            if (len(cnt) < 10):
                continue
            
            (x,y,w,h) = cv2.boundingRect(cnt)
            
            if w > 100 or h > 100:
                continue
            
            if w < 10 or h < 10:
                continue
            
            aspect_ratio = w/h if w<h else h/w
            if aspect_ratio < 0.35:
                continue
            
            cv2.rectangle(cv_image, (x,y), (x+w,y+h), (0,255,0), 2)
            bbox.append((x,y,w,h))
            
        bbox_msg = Int16MultiArray()
        for b in bbox:
            bbox_msg.data.append(b[0])
            bbox_msg.data.append(b[1])
            bbox_msg.data.append(b[2])
            bbox_msg.data.append(b[3])
        self.bbox_pub.publish(bbox_msg)
            
        cv2.imshow('image', cv_image)
        cv2.imshow('result', result)
        cv2.waitKey(1)
        
def main():
    rclpy.init()
    
    node = DetectionByThresholdNode()
    rclpy.spin(node)
    rclpy.shutdown()