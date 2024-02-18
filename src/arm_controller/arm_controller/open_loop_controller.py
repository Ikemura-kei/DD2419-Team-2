from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Int16MultiArray, MultiArrayDimension

# "{layout: {dim: [{label: '', size: 0, stride: 0}], data_offset: 0}, data: [12000,12000,12000,12000,12000,12000,500,500,500,500,500,500]}"

ANGLE_LIMITS_LOW = [4200, 3100, 3200, 3000, 3750, 1000]
ANGLE_LIMITS_HIGH = [12050, 21500, 20500, 23000, 17050, 22000]
ANGLE_HOMES = [4200, 12000, 12000, 12000, 12000, 12000]

ANGLE_PICKUP = [4200, 12000, 12000, 20601, 6242, 12000]
ANGLE_CLOSE_GRIPPER = [12050, 12000, 12000, 20601, 6242, 12000]
ANGLE_HOME_OBJ = [12050, 12000, 12000, 12000, 12000, 12000]

ANGLE_PLACE_DOWN = [12050, 12000, 12000, 17000, 10000, 12000]
ANGLE_OPEN_GRIPPER = [4200, 12000, 12000, 17000, 10000, 12000]

PERIOD = 0.1

DELAY = 5000 #ms

class OpenLoopController(Node):
    def __init__(self):
        super().__init__('open_loop_controller')
        
        self.joint_pos_sub = self.create_subscription(JointState, topic='/servo_pos_publisher', callback=self.joint_pos_cb, qos_profile=10)
        self.joycon_sub = self.create_subscription(Joy, topic='/joy', callback=self.joy_cb, qos_profile=10)
        self.joint_cmd_pub = self.create_publisher(Int16MultiArray, '/multi_servo_cmd_sub', 10)
        
        #ADDING FROM KINEMATICS STUFF. MAY DELETE LATER
        
        self.kinematics_sub = self.create_subscription(Int16MultiArray, topic='/kinematic_control', callback=self.kinematic_go_cb, qos_profile=10)
        
        
        self.last_joint_cmd_pub_time = self.get_clock().now()
        
        self.joint_cmd = Int16MultiArray()
        dim = MultiArrayDimension()
        dim.label = ''
        dim.size = 0
        dim.stride = 0
        self.joint_cmd.layout.dim.append(dim)
        self.joy_cmd = Joy()
        
        self.joint_angles = ANGLE_HOMES
        self.joint_times = [1000] * 6
        self.start_time = self.get_clock().now()
        self.init_cnt = 0
        
        self.joint_pos_sub
        
        #variables for pickup
        self.pickup_pressed = False
        self.gripper_closed = False
        self.delay_time = self.get_clock().now()
        
        self.place_pressed = False
        self.gripper_open = False
        self.place_pressed_time = self.get_clock().now()
        
    def joy_cb(self, msg:Joy):
        # axes: left-stick horizontal, left-stick verticle, LT, right-stick horizontal, right-stick verticle, RT, left-cross horizontal, left-cross verticle
        # buttons: A, B, X, Y, LB, RB, SACK, START
        self.joy_cmd = msg
        
        if (self.get_clock().now() - self.start_time).nanoseconds / 1e9 <= 3:
            if self.init_cnt < 3: # weired, requires 3 calls to start the motion
                print('init start')
                self.publish_joint_cmd(self.joint_angles, self.joint_times)
                self.init_cnt += 1
            print('still initializing')
            return
        
        
        #####################################################################################
        # RED BUTTON -- Reset
        #####################################################################################
        if self.joy_cmd.buttons[1]: # red button is pressed
            
            self.gripper_closed = False
            self.pickup_pressed = False
            self.pickup_pressed = False
            self.place_pressed = False
            self.gripper_open = False
            self.joint_angles = ANGLE_HOMES
            self.joint_times = [DELAY] * 6
            self.publish_joint_cmd(self.joint_angles, self.joint_times)
            print("Resetting home")
            return
        
        
        #####################################################################################
        # RED BUTTON -- Pick
        #####################################################################################
            
        if self.gripper_closed and ((self.get_clock().now() - self.delay_time).nanoseconds / 1e6 >= DELAY):
            self.gripper_closed = False
            self.joint_angles = ANGLE_HOME_OBJ
            self.joint_times = [DELAY] * 6
            self.publish_joint_cmd(self.joint_angles, self.joint_times)
            print("Returning home with object")
            return
            
        
        if self.pickup_pressed and ((self.get_clock().now() - self.delay_time).nanoseconds / 1e6 >= DELAY):
            self.pickup_pressed = False
            self.gripper_closed = True
            self.delay_time = self.get_clock().now()
            self.joint_angles= ANGLE_CLOSE_GRIPPER
            self.joint_times = [DELAY] * 6
            self.publish_joint_cmd(self.joint_angles, self.joint_times)
            print("Closing gripper")
            return
            
        if self.joy_cmd.buttons[0]: # green button is pressed (pickup!!!!!)
            self.pickup_pressed = True
            self.delay_time = self.get_clock().now()
            self.joint_angles = ANGLE_PICKUP
            self.joint_times = [DELAY] * 6
            self.publish_joint_cmd(self.joint_angles, self.joint_times)
            print("Moving arm down")
            return
        
        
        #####################################################################################
        # BLUE BUTTON -- Place
        #####################################################################################
        
        
        if self.gripper_open and ((self.get_clock().now() - self.delay_time).nanoseconds / 1e6 >= DELAY):
            self.gripper_open = False
            self.joint_angles = ANGLE_HOMES
            self.joint_times = [DELAY] * 6
            self.publish_joint_cmd(self.joint_angles, self.joint_times)
            print("Returning home")
            return
            
        
        if self.place_pressed and ((self.get_clock().now() - self.delay_time).nanoseconds / 1e6 >= DELAY):
            self.place_pressed = False
            self.gripper_open = True
            self.delay_time = self.get_clock().now()
            self.joint_angles= ANGLE_OPEN_GRIPPER
            self.joint_times = [DELAY] * 6
            self.publish_joint_cmd(self.joint_angles, self.joint_times)
            print("Opening gripper")
            return
        
        if self.joy_cmd.buttons[2]: # blue button is pressed (place!!!!!)
            self.place_pressed = True
            self.delay_time = self.get_clock().now()
            self.joint_angles = ANGLE_PLACE_DOWN
            self.joint_times = [DELAY] * 6
            self.publish_joint_cmd(self.joint_angles, self.joint_times)
            print("Moving arm down to place")
            return
    
    
    
    
    #ADDING FROM KINEMATICS STUFF. MAY DELETE LATER
    
    def kinematic_go_cb(self, msg:Int16MultiArray):
        
        self.joint_angles = [4200, msg.data[4], msg.data[3], msg.data[2], msg.data[1], msg.data[0]]
        self.joint_times = [DELAY] * 6
        print("Moving arm to {}".format(self.joint_angles))
        
        
        # return
        self.publish_joint_cmd(self.joint_angles, self.joint_times)
        return
    
        
    def joint_pos_cb(self, msg:JointState):
        assert len(msg.name) == 6
        
        self.positions = msg.position
        self.velocities = msg.velocity
        self.efforts = msg.effort
        
    def publish_joint_cmd(self, joint_angles, joint_times):
        self.joint_cmd.data = []
        assert len(joint_angles) == len(joint_times) and len(joint_times) == 6
        
        dt = (self.get_clock().now() - self.last_joint_cmd_pub_time).nanoseconds / 1e9 # in seconds

        if dt < PERIOD:
            return 
        self.last_joint_cmd_pub_time = self.get_clock().now()

        # print(dt)
        
        for i in range(6):
            if joint_angles[i] < ANGLE_LIMITS_LOW[i]:
                joint_angles[i] = ANGLE_LIMITS_LOW[i]
            elif joint_angles[i] > ANGLE_LIMITS_HIGH[i]:
                joint_angles[i] = ANGLE_LIMITS_HIGH[i]
            self.joint_cmd.data.append(joint_angles[i])
            
        for i in range(6):
            if joint_times[i] < 50:
                joint_times[i] = 50
            self.joint_cmd.data.append(joint_times[i])
                
        print(self.joint_cmd)
            
        self.joint_cmd_pub.publish(self.joint_cmd)

def main():
    print('Hi from arm_controller.')
    
    rclpy.init()
    
    open_loop_controller = OpenLoopController()
    
    rclpy.spin(open_loop_controller)
    
    open_loop_controller.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()