import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from robp_interfaces.msg import DutyCycles
from sensor_msgs.msg import Joy

SAFETY = 0.5

class DutyCyclePublisher(Node):
    def __init__(self):
        super().__init__("duty_cycle_publisher")

        self.joy_command = self.create_subscription(Joy,'/joy',self.joy_command_cb,10)
        self.joy_command

        self.pub = self.create_publisher(DutyCycles, '/motor/duty_cycles', 10)

        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.left_joy = 0.0 
        self.right_joy = 0.0

        
        
        # TODO: Implement
        
    def timer_callback(self):
        msg = DutyCycles()
        msg.duty_cycle_left = self.left_joy * SAFETY# Note that the left is harder to turn
        msg.duty_cycle_right = self.right_joy * .89 * SAFETY
        self.pub.publish(msg)
        self.get_logger().info('Left: {} Right: {}'.format(msg.duty_cycle_left,msg.duty_cycle_right))


    def joy_command_cb(self, msg):

        if msg.buttons[1]: #set duty cycle to zero if red button pressed
            self.left_joy = 0.0
            self.right_joy = 0.0
        
        else:
            self.left_joy = 0.5 - (round(msg.axes[2],3)/2)
            self.right_joy = 0.5 - (round(msg.axes[5],3)/2)

        # print("Left {}".format(self.left_joy))
        # print("Right {}".format(self.right_joy))
        
        #msg.axes --> has a list of the knobs (<class 'array.array'>)
        #msg.buttons --> has a list of the buttons (<class 'array.array'>)


        # msg = DutyCycles()
        # msg.duty_cycle_left = 1.0
        # msg.duty_cycle_right = 0.9
        # self.pub.publish(msg)

def main():
    print('Hi from controller.')

    rclpy.init()

    duty_cycle_publisher = DutyCyclePublisher()

    rclpy.spin(duty_cycle_publisher)

    duty_cycle_publisher.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
