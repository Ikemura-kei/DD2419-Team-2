import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import numpy as np

class FSMStates:
    INIT = 0
    WAITING = 1
    PICK_IN_PROC = 2
    WAIT_FOR_BOX_DET = 3
    GOING_TO_BOX = 4
    PLACE_IN_PROC = 5
    
    def __init__():
        pass
    
class MS1PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('ms1_pick_and_place_node')
        self.state = FSMStates.INIT
        
        self.joy_state = False
        self.last_button = False
        self.box_det_cnt = 0
        self.pick_counter = 0
        self.place_counter = 0
        self.box_reached = False
        self.start_go_to_box_time = self.get_clock().now()
        self.MIN_DET_CNT = 30
        self.MAX_PICK_CNT = 925
        self.MAX_PLACE_CNT = 925
        self.GO_TO_BOX_TIME_MAX = 10 # seconds
        self.MIN_DISTANCE = 0.155 # m
        
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.aruco_sub = self.create_subscription(MarkerArray, '/aruco/markers', self.aruco_cb, 10)
        
        self.goal_pub = self.create_publisher(Point, '/plan_goal', 10)
        self.last_joy_button = 0
        
    def joy_cb(self, msg:Joy):
        if self.last_joy_button != msg.buttons[3] and msg.buttons[3]:
            self.joy_state = not self.joy_state
            
        self.last_joy_button = msg.buttons[3]
        
    def aruco_cb(self, msg:MarkerArray):
        for aruco in msg.markers:
            if aruco.id != 1:
                continue
            
            self.box_det_cnt += 1
            self.target_x = aruco.pose.pose.position.z
            self.target_y = -aruco.pose.pose.position.x
            break
        
    def loop(self):
        # rate = self.create_rate(frequency=20)
        while rclpy.ok():
            # rate.sleep()
            print("==> State {}".format(self.state))
            print("==> Joy state {}".format(self.joy_state))
            rclpy.spin_once(self)
            
            if self.state == FSMStates.INIT:
                self.state = FSMStates.WAITING
                
            elif self.state == FSMStates.WAITING:
                if self.joy_state:
                    self.state = FSMStates.PICK_IN_PROC
                    self.pick_counter = 0
                    
            elif self.state == FSMStates.PICK_IN_PROC:
                if self.pick_counter < 1:
                    # -- trigger picking --
                    pass
                elif self.pick_counter > self.MAX_PICK_CNT:
                    # -- done picking --
                    self.state = FSMStates.WAIT_FOR_BOX_DET
                    self.box_det_cnt = 0
                    
                self.pick_counter += 1
                
            elif self.state == FSMStates.WAIT_FOR_BOX_DET:
                if self.box_det_cnt < self.MIN_DET_CNT:
                    pass
                else:
                    # -- valid box detected --
                    self.state = FSMStates.GOING_TO_BOX
                    self.box_reached = False
                    self.start_go_to_box_time = self.get_clock().now()
                    goal = Point()
                    goal.x = self.target_x
                    goal.y = self.target_y
                    # print("goal {}".format(goal))
                    self.goal_pub.publish(goal)
            
            elif self.state == FSMStates.GOING_TO_BOX:
                times_up = ((self.get_clock().now() - self.start_go_to_box_time).nanoseconds / 1e9) > self.GO_TO_BOX_TIME_MAX
                
                distance = np.sqrt(self.target_x**2 + self.target_y ** 2)
                self.box_reached = distance <= self.MIN_DISTANCE
                
                if self.box_reached or times_up:
                    # -- goal reached --
                    # -- stop --
                    self.state = FSMStates.PLACE_IN_PROC
                    self.place_counter = 0
                    
            elif self.state == FSMStates.PLACE_IN_PROC:
                self.place_counter += 1
                if self.place_counter > self.MAX_PLACE_CNT:
                    # -- done placing --
                    self.state = FSMStates.WAITING
                    self.joy_state = False
    
def main():
    rclpy.init()
    
    node = MS1PickAndPlaceNode()
    
    node.loop()
    
    node.destroy_node()
    
    rclpy.shutdown()



if __name__ == '__main__':
    main()