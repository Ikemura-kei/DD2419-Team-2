import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
from std_msgs.msg import Bool

class FSMStates:
    INIT = 0
    WAITING = 1
    GOING_TO_OBJ = 2
    FINISH_TIMEOUT = 3
    PICK = 4
    
    def __init__():
        pass
    
"""
Notes:
 - /plan_goal topic can be changed to a server or action server
 - 
"""
    
class MS2PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('ms2_pick_and_place_node')
        
        self.state = FSMStates.INIT
        
        self.obj_det_cnt = 0
        self.last_obj_det_time = self.get_clock().now()
        self.finish_time = self.get_clock().now()
        self.MIN_OBJ_DET_CNT = 2
        self.MAX_MISS_DUR = 5 # seconds
        self.FINISH_TIMEOUT = 3.5 # seconds
        self.MIN_OBJ_DISTANCE = 0.3 # m
        self.goal_reached = False
        self.obj_position = Point()
        self.MAX_SEND_GOAL_TIMEOUT = 0.45
        
        self.goal_pub = self.create_publisher(Point, '/plan_goal', 10) 
        
        self.object_sub = self.create_subscription(MarkerArray, '/object_centers', self.obj_cb, 1) #in odom frame
        
        self.joycon_sub = self.create_subscription(Joy, topic='/joy', callback=self.joy_cb, qos_profile=10) 
        self.commence = False
        
        self.goal_reached_sub = self.create_subscription(Bool, topic='/goal_reached', callback=self.goal_reached_cb, qos_profile=10)
        self.goal_reached = False
        
    def goal_reached_cb(self, msg):
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
        
        # print("==> Received number of markers {}".format(len(msg.markers)))
        for marker in msg.markers:
            if not 'blue' in marker.ns:
                continue

            self.obj_position.x = marker.pose.position.x
            self.obj_position.y = marker.pose.position.y
            self.last_obj_det_time = self.get_clock().now()
            self.obj_det_cnt += 1
            # print("==> Object detection counter:", self.obj_det_cnt)
            
            break
        
        
    def loop(self):
        print("==> State: {}".format(self.state))
        
        if not (self.commence):
            print("NOOOOOOOOOOO LOOOOOOOOOOOOOOOOOOOOOOOOPPPPPPPPPPPPPPPPP")
            return 
        
        if self.state == FSMStates.INIT:
            self.state = FSMStates.WAITING
            self.obj_det_cnt = 0
            
        elif self.state == FSMStates.WAITING:
            if self.obj_det_cnt >= self.MIN_OBJ_DET_CNT:
                self.state = FSMStates.GOING_TO_OBJ
                self.goal_pub.publish(self.obj_position)
                
        elif self.state == FSMStates.GOING_TO_OBJ:
            distance_to_obj = np.sqrt(self.obj_position.x ** 2 + self.obj_position.y ** 2)
            self.goal_reached = distance_to_obj < self.MIN_OBJ_DISTANCE # if the distance of the detected object is smaller than a threshold, we confirm reached
            print("==> Distance to object {}".format(distance_to_obj))
            if ((self.get_clock().now() - self.last_obj_det_time).nanoseconds / 1e9) > self.MAX_MISS_DUR:
                self.state = FSMStates.WAITING
                self.obj_det_cnt = 0
            elif self.goal_reached:
                # -- stop --
                self.state = FSMStates.FINISH_TIMEOUT
                self.finish_time = self.get_clock().now()
            elif ((self.get_clock().now() - self.last_obj_det_time).nanoseconds / 1e9) < self.MAX_SEND_GOAL_TIMEOUT:
                self.goal_pub.publish(self.obj_position)
        
        elif self.state == FSMStates.FINISH_TIMEOUT:
            if ((self.get_clock().now() - self.finish_time).nanoseconds / 1e9) > self.FINISH_TIMEOUT:
                self.state = FSMStates.PICK
                # pass
                
        elif self.state == FSMStates.PICK:
            if ((self.get_clock().now() - self.finish_time).nanoseconds / 1e9) > self.FINISH_TIMEOUT:
                self.state = FSMStates.WAITING
                # pass
                
            
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