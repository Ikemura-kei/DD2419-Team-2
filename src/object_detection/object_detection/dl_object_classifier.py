#!/usr/bin/env python
import rclpy #ROS2 client library for Python.

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point
from rclpy.node import Node
import torch #imports the PyTorch library, which is used for deep learning tasks.
from PIL import Image as pil_image #imports the Image class from the Python Imaging Library (PIL) and renames it as pil_image.

from object_detection.dl_detection import Detector
import numpy as np
from cv_bridge import CvBridge, CvBridgeError #These are used for converting between ROS Image messages and OpenCV images.
import cv2 #This imports the OpenCV library, which is used for computer vision tasks.
from dl_perception_interfaces.msg import BoundingBox, BoundingBoxArray
import time
import os
import message_filters

if torch.cuda.is_available():
    os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "max_split_size_mb:512"
    torch.cuda.set_per_process_memory_fraction(0.6,0)
    torch.cuda.empty_cache()


class Object_classifier(Node):

    def __init__(self):
        super().__init__('object_classifier')
       
        # Define rate
        # self.update_rate = 10 # [Hz] Change this to the rate you want
        # self.update_dt = 1.0/self.update_rate # [s]
        # self.rate = rclpy.Rate(self.update_rate) 
        # rclpy.loginfo(torch.__version__)
        
        # Paramethers 
        if torch.cuda.is_available():
            self.device = "cuda"
        else:
            self.device = "cpu"  # Fallback to CPU if CUDA is not available

        detector = Detector()
        model_path = "/home/team2/dd2419_ws/src/object_detection/object_detection/dl_detection_model_weights/det_2023-03-31_10-42-53-672891.pt" #for robot
        example_forward_input = torch.rand(8, 3, 640, 480)
        self.model = self.load_model(detector, model_path, self.device)
        self.model_opt = torch.jit.trace(self.model, example_forward_input).to(self.device)
        self.model_opt.eval()
        


        self.bridge = CvBridge()

        self.mapping = ["Binky", "Hugo", "Slush", "Muddles", "Kiki", "Oakie", "Cube", "Sphere"]
        self.sub_mapping_cube = ["Red_cube", "Green_cube", "Blue_cube", "Wooden_cube"]
        self.sub_mapping_sphere = ["Red_ball", "Green_ball", "Blue_ball"]
        self.depth = None
        
        self.camera_info = [None, None, None, None]
        
        # Subscribers 
        # self.sub_image = rclpy.Subscriber("/camera/color/image_raw", Image, self.image_callback) 
        # self.sub_depth= rclpy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)
        self.sub_camera_info = self.create_subscription(
            CameraInfo, 
            "/camera/color/camera_info", 
            self.camera_info_callback, 
            10
        )

        # image_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        # depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
        # ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 2)
        # ts.registerCallback(self.combined_callback)

        # image_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        # depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
        # self.ts = message_filters.TimeSynchronizer([image_sub, depth_sub], 10)
        # self.ts.registerCallback(self.combined_callback)

        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_image_callback,
            10)
        
        # Publisher
        self.bb_pub = self.create_publisher(
            BoundingBoxArray, 
            "/detection/bounding_boxes", 
            10)
        
        self.image_bb_pub = self.create_publisher(
            Image, 
            "/detection/image_with_bounding_boxes", 
            10)

        
        
    # def combined_callback(self, image_msg, depth_msg): 
    #     try:
    #         t0 = time.time()
    #         cv_image = self.bridge.imgmsg_to_cv2(image_msg, "rgb8")
    #         depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
    #         np_arr = np.asarray(depth_image)
    #         depth = np_arr
    #         self.compute_bb(image_msg.header.stamp, image_msg.header.frame_id, depth, cv_image, t0) 
    #         # This compute_bb method computes bounding boxes and processes the images for object detection.
    #     except CvBridgeError as e:
    #         print(e)

    
    def image_callback(self, msg): 
        """Callback function for the topic"""
        try:
            t0 = time.time()
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

            
            if self.depth is not None:
                self.compute_bb(msg.header.stamp, msg.header.frame_id, self.depth, cv_image, t0) 

        except CvBridgeError as e:
            print(e)
            


    def depth_image_callback(self, msg): 
        """Callback function for the topic"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            np_arr = np.asarray(cv_image)
            self.depth = np_arr

        except CvBridgeError as e:
            print(e)



    def camera_info_callback(self, msg): 
        """Callback function for the topic"""
        
        self.camera_info[0] = msg.k[0] #fx
        self.camera_info[1] = msg.k[4] #fy
        self.camera_info[2] = msg.k[2] #cx
        self.camera_info[3] = msg.k[5] #cy
        
        # **Intrinsic camera matrix for the raw (distorted) images.**
            #      [fx  0 cx]
            #  K = [ 0 fy cy]
            #      [ 0  0  1]
            #  Projects 3D points in the camera coordinate frame to 2D pixel
            #  coordinates using the focal lengths (fx, fy) and principal point (cx, cy).
            # parameters are used later in the code for computing 3D coordinates from pixel coordinates using the camera model.




    def load_model(self,model: torch.nn.Module, path: str, device: str) -> torch.nn.Module:
        """Load model weights from disk.

        Args:
            model: The model to load the weights into.
            path: The path from which to load the model weights.
            device: The device the model weights should be on.

        Returns:
            The loaded model (note that this is the same object as the passed model).
        """
        state_dict = torch.load(path, map_location=device)
        model.load_state_dict(state_dict)
        return model



    def compute_bb(self, stamp, frame_id, depth_image, cv_image, t0):     
        
        np_arr = np.asarray(cv_image)
        counter = 0
        
        test_images = []
        torch_image  = self.model.input_transform_inference(cv_image)
        test_images.append(torch_image)
        if torch.cuda.is_available():
            test_images = torch.stack(test_images)
        else:
            test_images = torch.stack(test_images).to(self.device)

        test_image = test_images.to(self.device)
        
        with torch.no_grad():
            out = self.model_opt(test_image).cpu()
            bbs = self.model.decode_output(out, 0.85)

            
            bb_list_msg = BoundingBoxArray()
            bb_list_msg.header.stamp = stamp
            bb_list_msg.header.frame_id = frame_id
           
            
            for bb in bbs[0]:
                
                x_bb = int(bb["x"])
                y_bb = int(bb["y"])
                
                bb_msg = BoundingBox()
                bb_msg.stamp = stamp
                bb_msg.x = float(x_bb)
                bb_msg.y = float(y_bb)
                bb_msg.width = bb["width"]
                bb_msg.height = bb["height"]
                bb_msg.category_id = bb["category"]
                
                # Call function to compute point and depth
                x,y,depth = self.compute_point(bb, depth_image)

                if bb["category"] == 6 or bb["category"] == 7:
                    bb_msg.category_name = self.compute_color(bb, np_arr)     
                else:
                    bb_msg.category_name = self.mapping[bb["category"]]
                
                if bb_msg.category_name is not None and depth > 0.15 and depth < 2 and bb["width"]*bb["width"] > 100:
                    
                    # visualize image with bb in Rviz
                    start_point = (x_bb, y_bb)
                    end_point = (int(x_bb+bb["width"]), int(y_bb+bb["height"]))
                    color = (255, 0, 0)
                    thickness = 2
                    cv_image = cv2.rectangle(cv_image, start_point, end_point, color, thickness)
                    cv_image = cv2.putText(cv_image, bb_msg.category_name, (start_point[0]-10, start_point[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 
                    1, color, thickness, cv2.LINE_AA)
                    imgMsg = self.bridge.cv2_to_imgmsg(cv_image, "rgb8")
                    self.image_bb_pub.publish(imgMsg)
                    
                    
                    point = Point()
                    point.x = x
                    point.y = y
                    point.z = depth
                    bb_msg.bb_center = point
                    bb_list_msg.bounding_boxes.append(bb_msg)

                
            if len(bb_list_msg.bounding_boxes)>0:
                
                print("one bounding_boxes published: ", str(bb_msg.category_name),"           (with Box Position:", bb_msg.x, bb_msg.y)
                
                self.bb_pub.publish(bb_list_msg)
              
            # tinfer = time.time() - t0
            # rclpy.loginfo(1/tinfer)



    def compute_point(self, bb, depth_image):
        depth = depth_image[int(bb["y"]+bb["height"]/2),int(bb["x"]+bb["width"]/2)]/1000

        x=0
        y=0
        if self.camera_info is not None:      
            x = depth*(int(bb["x"]+bb["width"]/2) - self.camera_info[2]) / self.camera_info[0] 
            y = depth*(int(bb["y"]+bb["height"]/2) - self.camera_info[3]) / self.camera_info[1]
        
        return x, y, depth
    


    def compute_color(self, bb, image):
        
        # Crop image
        cropped_image = image[int(bb["y"]):int(bb["y"]+bb["height"]),int(bb["x"]):int(bb["x"]+bb["width"])]
        mean_red = np.mean(cropped_image[:,:,0])
        mean_green = np.mean(cropped_image[:,:,1])
        mean_blue = np.mean(cropped_image[:,:,2])
    
        category_id = bb["category"]
       
        max_color = max(mean_red, mean_blue, mean_green)
        mapping = None
        if category_id == 6:
            mapping = self.sub_mapping_cube
        else:
            mapping = self.sub_mapping_sphere

        category_name = "_"
        
        metric = np.std([mean_red, mean_green, mean_blue])/np.mean([mean_red, mean_green, mean_blue])

        if category_id == 6 and  metric < 0.17 and max_color != mean_green:
            category_name = mapping[3]
        elif max_color == mean_green:
            category_name = mapping[1]
        elif max_color == mean_blue:
            category_name = mapping[2]
        elif  max_color == mean_red:
            category_name = mapping[0]
        else: 
            category_name = "_None after color computation"
        
        #rclpy.loginfo("%s, %s, %s, %s",mean_red, mean_green, mean_blue, metric)
        return category_name

        
           


                

def main():
    rclpy.init()
    node = Object_classifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()








