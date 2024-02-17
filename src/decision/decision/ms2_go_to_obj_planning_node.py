import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point, PoseArray
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np

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
        
        self.obj_det_cnt = 0
        self.last_obj_det_time = self.get_clock().now()
        self.finish_time = self.get_clock().now()
        self.MIN_OBJ_DET_CNT = 2
        self.MAX_MISS_DUR = 5 # seconds
        self.FINISH_TIMEOUT = 3.5 # seconds
        self.MIN_OBJ_DISTANCE = 0.3 # m
        self.goal_reached = False
        self.intermediate_goal_reached = False
        self.obj_position = Point()
        self.intermediate_goal = Point()
        self.planned_poses = PoseArray()
        self.MAX_SEND_GOAL_TIMEOUT = 0.45
        
        self.goal_pub = self.create_publisher(Point, '/plan_goal', 10)
        self.target_position_pub = self.create_publisher(Point, '/target_position', 10)
        
        self.object_sub = self.create_subscription(MarkerArray, '/object_centers', self.obj_cb, 1)

        self.object_sub = self.create_subscription(PoseArray, '/planned_poses', self.poses_cb, 10)

        self.completion_sub = self.create_subscription(
            String,
            'task_completion',
            self.completion_callback,
            10)
    
    def completion_callback(self, msg: String):
        self.get_logger().info('==> Task completed')
        if len(self.planned_poses.poses) == 0:
            self.goal_reached = True
        else:
            self.intermediate_goal_reached = True
        self.obj_det_cnt = 0
        
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
        
    def poses_cb(self, msg: PoseArray):
        self.planned_poses = msg
        print("==> Received pose %d" % len(msg.poses))



    def loop(self):
        print("==> State: {}".format(self.state))
        
        if self.state == FSMStates.INIT:
            self.state = FSMStates.WAITING
            self.obj_det_cnt = 0
            
        elif self.state == FSMStates.WAITING:
            # To uncomment when linked with object detection
            #if self.obj_det_cnt >= self.MIN_OBJ_DET_CNT:
                #self.state = FSMStates.COMPUTING_PATH
                #self.goal_pub.publish(self.obj_position)
            
            # To remove when linked with object detection
            self.state = FSMStates.COMPUTING_PATH
            self.obj_position.x = 4.3
            self.obj_position.y = 1.5
            self.get_logger().info('==> Sending goal')
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
            #distance_to_obj = np.sqrt(self.obj_position.x ** 2 + self.obj_position.y ** 2)
            #distance_to_intermediate_goal = np.sqrt(self.intermediate_goal.x ** 2 + self.intermediate_goal.y ** 2)
            #self.goal_reached = distance_to_obj < self.MIN_OBJ_DISTANCE # if the distance of the detected object is smaller than a threshold, we confirm reached
            #self.intermediate_goal_reached = distance_to_intermediate_goal < self.MIN_OBJ_DISTANCE
            #print("==> Distance to object {}".format(distance_to_obj))
            #if ((self.get_clock().now() - self.last_obj_det_time).nanoseconds / 1e9) > self.MAX_MISS_DUR:
                #self.state = FSMStates.WAITING
                #self.obj_det_cnt = 0
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
                #self.state = FSMStates.WAITING
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