import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point, PoseArray
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
from nav_msgs.msg import OccupancyGrid

from std_msgs.msg import String

class FSMStates:
    INIT = 0
    WAITING = 1
    COMPUTING_PATH = 2
    GOING_TO_OBJ = 3
    FINISH_TIMEOUT = 4
    
    def __init__():
        pass
    
class MS1GoToDetectedObjNodeV2(Node):
    def __init__(self):
        super().__init__('go_to_detected_obj_node')
        
        self.state = FSMStates.INIT
        self.joy_state = False
        
        self.last_obj_det_time = self.get_clock().now()
        self.finish_time = self.get_clock().now()
        self.MIN_OBJ_DET_CNT = 2
        self.MAX_MISS_DUR = 5 # seconds
        self.FINISH_TIMEOUT = 3.5 # seconds
        self.MIN_OBJ_DISTANCE = 0.3 # m
        self.goal_reached = False
        self.intermediate_goal_reached = False
        self.obj_position = Point()
        self.map = OccupancyGrid()
        self.intermediate_goal = Point()
        self.planned_poses = PoseArray()
        self.MAX_SEND_GOAL_TIMEOUT = 0.45
        
        self.goal_pub = self.create_publisher(Point, '/plan_goal', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.target_position_pub = self.create_publisher(Point, '/target_position', 10)
        
        self.map_sub = self.create_subscription(OccupancyGrid, '/global_map', self.map_cb, 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.object_sub = self.create_subscription(PoseArray, '/planned_poses', self.poses_cb, 10)

        self.completion_sub = self.create_subscription(
            String,
            'task_completion',
            self.completion_callback,
            10)
    
    def joy_cb(self, msg:Joy):
        print("==> Joy callback")
        if msg.buttons[1]:
            self.joy_state = False 
        elif msg.buttons[0]:  
            self.joy_state = True 
            
    
    def completion_callback(self, msg: String):
        self.get_logger().info('==> Task completed')
        if len(self.planned_poses.poses) == 0:
            self.goal_reached = True
        else:
            self.intermediate_goal_reached = True
        self.obj_det_cnt = 0
        
    def map_cb(self, msg:OccupancyGrid):
        self.map = msg
        
    def poses_cb(self, msg: PoseArray):
        self.planned_poses = msg
        print("==> Received pose %d" % len(msg.poses))



    def loop(self):
        print("==> State: {}".format(self.state))
        print("==> Joy state {}".format(self.joy_state))
        
        if self.state == FSMStates.INIT:
            self.state = FSMStates.WAITING
            
        elif self.state == FSMStates.WAITING:
            if len(self.map.data) > 0 and self.joy_state:
                self.state = FSMStates.COMPUTING_PATH
                self.obj_position.x = 2.0
                self.obj_position.y = 0.0
                self.get_logger().info('==> Sending goal')
                for data in self.map.data:
                    print(data)
                self.map_pub.publish(self.map)
                self.goal_pub.publish(self.obj_position)
        
        elif self.state == FSMStates.COMPUTING_PATH:
            if len(self.planned_poses.poses) > 0:
                self.state = FSMStates.GOING_TO_OBJ
                self.get_logger().info('==> Going to object')
                self.intermediate_goal = self.planned_poses.poses.pop().position
                self.target_position_pub.publish(self.intermediate_goal)
                self.get_logger().info('==> Sending first intermediate goal')
            #TO-DO: add a timeout to go back to WAITING state if no path is received
                
                
        elif self.state == FSMStates.GOING_TO_OBJ:
            if self.goal_reached:
                # -- stop --
                self.state = FSMStates.FINISH_TIMEOUT
                self.finish_time = self.get_clock().now()
            elif self.intermediate_goal_reached:
                self.intermediate_goal_reached = False
                self.intermediate_goal = self.planned_poses.poses.pop().position
                self.target_position_pub.publish(self.intermediate_goal)
                self.get_logger().info('==> Sending intermediate goal')
            #elif ((self.get_clock().now() - self.last_obj_det_time).nanoseconds / 1e9) < self.MAX_SEND_GOAL_TIMEOUT:
                #self.goal_pub.publish(self.obj_position)
        
        elif self.state == FSMStates.FINISH_TIMEOUT:
            if ((self.get_clock().now() - self.finish_time).nanoseconds / 1e9) > self.FINISH_TIMEOUT:
                self.get_logger().info('==> Timeout finished, should restart a new loop setting state to FSMStates.WAITING')
                self.state = FSMStates.WAITING
                self.joy_state = False
                # pass
                
            
def main():
    rclpy.init()
    
    node = MS1GoToDetectedObjNodeV2()
    
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