from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Int16MultiArray, MultiArrayDimension

# "{layout: {dim: [{label: '', size: 0, stride: 0}], data_offset: 0}, data: [12000,12000,12000,12000,12000,12000,500,500,500,500,500,500]}"

ANGLE_LIMITS_LOW = [200, 3100, 3200, 3000, 750, 1000]
ANGLE_LIMITS_HIGH = [12050, 21500, 20500, 23000, 17050, 22000]
ANGLE_HOMES = [4200, 12000, 12000, 12000, 12000, 12000]
PERIOD = 0.1

class JoyStickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')
        
        self.joint_pos_sub = self.create_subscription(JointState, topic='/servo_pos_publisher', callback=self.joint_pos_cb, qos_profile=10)
        self.joycon_sub = self.create_subscription(Joy, topic='/joy', callback=self.joy_cb, qos_profile=10)
        self.joint_cmd_pub = self.create_publisher(Int16MultiArray, '/multi_servo_cmd_sub', 10)
        
        
        #ADDED FOR DEBUGGING PURPOSES
        self.joint_cmd_sub = self.create_subscription(Int16MultiArray, '/multi_servo_cmd_sub', callback=self.joint_cmd_cb, qos_profile=10)
        
        self.last_joint_cmd_pub_time = self.get_clock().now()
        
        self.joint_cmd = Int16MultiArray()
        dim = MultiArrayDimension()
        dim.label = ''
        dim.size = 0
        dim.stride = 0
        self.joint_cmd.layout.dim.append(dim)
        self.joy_cmd = Joy()
        
        self.joint_angles = ANGLE_HOMES
        self.joint_times = [1500] * 6
        self.start_time = self.get_clock().now()
        self.init_cnt = 0
        
        self.joint_pos_sub
        
        
        #ADDED FOR DEBUGGING PURPOSES
        self.prev_state = []
        
    
    
    
    
     #ADDED FOR DEBUGGING PURPOSES
    def joint_cmd_cb(self, msg:Int16MultiArray):
        assert len(msg.data) == 12
        
        
        self.joint_angles[0] = msg.data[0]
        self.joint_angles[1] = msg.data[1]
        self.joint_angles[2] = msg.data[2]
        self.joint_angles[3] = msg.data[3]
        self.joint_angles[4] = msg.data[4]
        self.joint_angles[5] = msg.data[5]
        
        print(self.joint_angles)
        
        self.prev_state = self.joint_angles
    
    
    
        
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
        
        if self.joy_cmd.buttons[5] < 1:
            return
        
        if self.joy_cmd.axes[2] > 0.999:
            self.joy_cmd.axes[2] = 1
        elif self.joy_cmd.axes[2] < -0.999:
            self.joy_cmd.axes[2] = -1
        self.joy_cmd.axes[2] = -self.joy_cmd.axes[2] + 1
            
        if self.joy_cmd.axes[5] > 0.999:
            self.joy_cmd.axes[5] = 1
        elif self.joy_cmd.axes[5] < -0.999:
            self.joy_cmd.axes[5] = -1
        self.joy_cmd.axes[5] = -self.joy_cmd.axes[5] + 1
        
        self.joint_angles[0] += int(self.joy_cmd.axes[6] * 70) # left-cross horizontal
        self.joint_angles[2] += int(self.joy_cmd.axes[7] * 100) # left-cross verticle
        self.joint_angles[5] += int(self.joy_cmd.axes[3] * 250) # right-stick horizontal
        self.joint_angles[3] += int(self.joy_cmd.axes[4] * 70) # right-stick verticle
        self.joint_angles[1] += int(self.joy_cmd.axes[1] * 300) # left-stick verticle
        self.joint_angles[4] += int(self.joy_cmd.axes[2] * 35) - int(self.joy_cmd.axes[5] * 35) # LT & RT
        self.joint_times = [90] * 6
            
        self.publish_joint_cmd(self.joint_angles, self.joint_times)
        
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
    print('Hi from joystick_controller.')
    
    rclpy.init()
    
    joystick_controller = JoyStickController()
    
    rclpy.spin(joystick_controller)
    
    joystick_controller.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()