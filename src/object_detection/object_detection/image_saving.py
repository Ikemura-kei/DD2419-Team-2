import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # Change this to the actual camera topic
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.save_dir = '/home/team2/DD2419-Team-2/src/object_detection/object_detection/data_set'  # Change this to the directory where you want to save images
        os.makedirs(self.save_dir, exist_ok=True)
        self.image_count = 0
        self.last_saved_time = time.time()

    def image_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_saved_time >= 1:  # Save image every 3 seconds
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception as e:
                self.get_logger().info(f"Failed to convert ROS Image to OpenCV: {e}")
                return

            # Save the image
            image_filename = os.path.join(self.save_dir, f"image2_{self.image_count}.png")
            cv2.imwrite(image_filename, cv_image)
            self.get_logger().info(f"Saved image: {image_filename}")
            self.image_count += 1
            self.last_saved_time = current_time

def main(args=None):
    rclpy.init(args=args)
    image_saver_node = ImageSaverNode()
    rclpy.spin(image_saver_node)
    image_saver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
