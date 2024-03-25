import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import MarkerArray
from aruco_msgs.msg import MarkerArray as ArucoMarkerArray
import numpy as np
from std_msgs.msg import Bool
from rclpy.callback_groups import ReentrantCallbackGroup

class FSMStates:
    INIT = 0
    WAITING = 1
    GOING_TO_OBJ = 2
    REACHED_TIMEOUT = 3
    PICK = 4
    PICK_RES = 5
    WAIT_FOR_BOX_DET = 6
    GOING_TO_BOX = 7
    PLACE_IN_PROC = 8
    
    def __init__():
        pass
    
"""
Notes:
 - /plan_goal topic can be changed to a server or action server
    - it is already in odom frame
 - aruco sub has frame camera_color_optical_frame
 
 CURRENT IMPLEMENTATION TAKES THE DETECTIONS WHEN ROBOT IS FIXED (I.E. ODOM AND BASE_LINK ARE TOGETHER)
"""
    
class MS2PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('ms2_pick_and_place_node')
        
        self.state = FSMStates.INIT
        
        self.obj_det_cnt = 0
        self.last_obj_det_time = self.get_clock().now()
        self.finish_time = self.get_clock().now()
        self.MIN_OBJ_DET_CNT = 2
        self.MIN_DET_CNT = 2
        self.MAX_MISS_DUR = 20 # seconds
        self.REACHED_TIMEOUT = 3.0 # seconds
        self.MIN_OBJ_DISTANCE = 0.3 # m
        self.goal_reached = False
        self.obj_position = PointStamped()
        self.MAX_SEND_GOAL_TIMEOUT = 0.45
        
        cbg1 = ReentrantCallbackGroup()
        
        self.goal_pub = self.create_publisher(PointStamped, '/plan_goal', 10) 
        
        self.object_sub = self.create_subscription(MarkerArray, '/object_centers', self.obj_cb, 10)#, callback_group=cbg1) #in odom frame
        
        self.joycon_sub = self.create_subscription(Joy, topic='/joy', callback=self.joy_cb, qos_profile=10) 
        self.commence = False
        
        self.goal_reached_sub = self.create_subscription(Bool, topic='/goal_reached', callback=self.goal_reached_cb, qos_profile=10)
        self.goal_reached = False
        
        self.pick_pub = self.create_publisher(PointStamped, '/pick_ik', 10)
        self.pick_goal = PointStamped()
        
        self.ik_res_sub = self.create_subscription(Bool, '/ik_res', self.ik_res_cb, 10)
        self.ik_success = False
        self.msg_received = False
        
        self.update_obj = True # this is to only update the detection during waitings
        
        self.aruco_sub = self.create_subscription(ArucoMarkerArray, '/aruco/markers', self.aruco_cb, 10)#, callback_group=cbg1)
        
        self.update_aruco = True
        self.box_det_cnt = 0
        
        self.drop_pub = self.create_publisher(Bool, '/drop_obj', 10)
        self.drop_obj_sub = self.create_subscription(Bool, '/drop_obj_res', self.drop_obj_cb, 10)
        self.drop_success = False
        self.drop_msg_received = False  
    
    
    def drop_obj_cb(self, msg: Bool):
        self.drop_success = msg.data
        self.drop_msg_received = True       
    
    # this needs to change. It is being put in the odom frame manually. Should be using transform!
    def aruco_cb(self, msg:ArucoMarkerArray):
        
        print("In aruco cb!")
        if not self.update_aruco:
            return
        
        for aruco in msg.markers:
            # if aruco.id != 1:
            #     continue
            
            self.box_det_cnt += 1
            self.target_x = aruco.pose.pose.position.z
            self.target_y = -aruco.pose.pose.position.x
            print(self.target_x, self.target_y)
            break
    
    def ik_res_cb(self, msg:Bool):
        self.ik_success = msg.data
        self.msg_received = True
    
    def ik_res_cb(self, msg:Bool):
        self.ik_success = msg.data
        self.msg_received = True
    
    def goal_reached_cb(self, msg:Bool):
        self.goal_reached = msg.data 
    
    def joy_cb(self, msg:Joy):
        # axes: left-stick horizontal, left-stick verticle, LT, right-stick horizontal, right-stick verticle, RT, left-cross horizontal, left-cross verticle
        # buttons: A, B, X, Y, LB, RB, SACK, START
        self.joy_cmd = msg
        
        if self.joy_cmd.buttons[0]: # if A is pressed. Start
            self.commence = True
        
        if self.joy_cmd.buttons[3]: # if Y is pressed. Stop
            self.commence = False
        
    def obj_cb(self, msg:MarkerArray):
        
        print("In OBJECT cb!")
        if not self.update_obj:
            return
        # print("==> Received number of markers {}".format(len(msg.markers)))
        for marker in msg.markers:
            if not 'blue' in marker.ns:
                continue
            
            self.obj_position.header.frame_id = marker.header.frame_id
            self.obj_position.header.stamp = marker.header.stamp
            self.obj_position.point.x = marker.pose.position.x
            self.obj_position.point.y = marker.pose.position.y
            
            
            # self.obj_position.x = marker.pose.position.x
            # self.obj_position.y = marker.pose.position.y
            
            self.last_obj_det_time = self.get_clock().now()
            self.obj_det_cnt += 1
            # print("==> Object detection counter:", self.obj_det_cnt)
            
            self.pick_goal.header.stamp = marker.header.stamp
            self.pick_goal.header.frame_id = marker.header.frame_id
            self.pick_goal.point.x = marker.pose.position.x
            self.pick_goal.point.y = marker.pose.position.y
            
            break
        
        
    def loop(self):
        # print("==> State: {}".format(self.state))
        
        if not (self.commence):
            # print("NOOOOOOOOOOO LOOOOOOOOOOOOOOOOOOOOOOOOPPPPPPPPPPPPPPPPP")
            return 
        
        if self.state == FSMStates.INIT:
            self.state = FSMStates.WAITING
            self.box_det_cnt = 0
            self.obj_det_cnt = 0
            self.update_obj = True
            self.update_aruco = True
            
        elif self.state == FSMStates.WAITING:
            self.update_obj = True
            self.update_aruco = True
            if self.obj_det_cnt >= self.MIN_OBJ_DET_CNT:
                self.state = FSMStates.GOING_TO_OBJ
                self.update_obj = False
                # self.update_aruco = False # do this because when robot moves then aruco detections are incorrect!
                self.goal_pub.publish(self.obj_position)
                
        elif self.state == FSMStates.GOING_TO_OBJ:
            # distance_to_obj = np.sqrt(self.obj_position.x ** 2 + self.obj_position.y ** 2)
            # self.goal_reached = distance_to_obj < self.MIN_OBJ_DISTANCE # if the distance of the detected object is smaller than a threshold, we confirm reached
            # print("==> Distance to object {}".format(distance_to_obj))
            if self.goal_reached:
                self.state = FSMStates.REACHED_TIMEOUT
                self.goal_reached = False
                # -- stop --
                # self.finish_time = self.get_clock().now()
            # elif ((self.get_clock().now() - self.last_obj_det_time).nanoseconds / 1e9) > self.MAX_MISS_DUR:
            #     self.state = FSMStates.WAITING
            #     self.obj_det_cnt = 0
            # elif ((self.get_clock().now() - self.last_obj_det_time).nanoseconds / 1e9) < self.MAX_SEND_GOAL_TIMEOUT: # may want to see object a few times before publishing again. Check this
            #     pass
                # self.goal_pub.publish(self.obj_position)
        
        elif self.state == FSMStates.REACHED_TIMEOUT:
            if ((self.get_clock().now() - self.finish_time).nanoseconds / 1e9) > self.REACHED_TIMEOUT:
                self.state = FSMStates.PICK
                # pass
                
        elif self.state == FSMStates.PICK:
            # self.pick_goal.header.stamp = self.get_clock().now().to_msg()
                self.pick_pub.publish(self.pick_goal)
                self.state = FSMStates.PICK_RES
            
        elif self.state == FSMStates.PICK_RES:
            if self.msg_received:
                self.msg_received = False
                if self.ik_success:
                    self.state = FSMStates.WAIT_FOR_BOX_DET
                else:
                    self.state = FSMStates.WAITING
            
        elif self.state == FSMStates.WAIT_FOR_BOX_DET:
            # if self.box_det_cnt <= self.MIN_DET_CNT:
            #     pass
            # else:
            # -- valid box detected --
            self.update_aruco = False
            self.state = FSMStates.GOING_TO_BOX
            goal = PointStamped()
            goal.point.x = self.target_x + self.obj_position.point.x - 0.06
            goal.point.y = self.target_y + self.obj_position.point.y
            goal.header.frame_id = 'odom'
            goal.header.stamp = self.get_clock().now().to_msg()
            # print("goal {}".format(goal))
            self.goal_pub.publish(goal)
        
        elif self.state == FSMStates.GOING_TO_BOX:
            if self.goal_reached:
                self.state = FSMStates.PLACE_IN_PROC
                self.goal_reached = False
                drop = Bool()
                drop.data = True
                self.drop_pub.publish(drop)
        
        elif self.state == FSMStates.PLACE_IN_PROC:
            if self.drop_msg_received:
                self.drop_msg_received = False
                if self.drop_success:
                    pass
                else:
                    pass
            
            
def main():
    rclpy.init()
    
    node = MS2PickAndPlaceNode()
    
    rate = node.create_rate(20, node.get_clock())
    while rclpy.ok():
        # rate.sleep()
        # print('sleep')
        rclpy.spin_once(node)
        node.loop()
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()