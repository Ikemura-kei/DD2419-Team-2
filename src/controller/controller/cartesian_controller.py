#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from robp_interfaces.msg import DutyCycles, Encoders
from geometry_msgs.msg import Twist

import math


class CartesianController(Node):

    def __init__(self):
        super().__init__('cartesian_controller')
        
        self.desired_v = 0
        self.desired_w = 0
        self.desired_wr = 0
        self.desired_wl = 0
        
        self.correcting_term_r = 0.95
        
        self.radius = 9.8 * 0.01 / 2
        self.base = 31 * 0.01

        self.actual_wr = 0
        self.actual_wl = 0

        self.prev_error_r = 0
        self.prev_error_l = 0
        self.dt = 0.1
        self.int_error_r = 0
        self.int_error_l = 0
        self.d_error_r = 0
        self.d_error_l = 0
        
        self.alpha_r = 0.75123
        self.beta_r =  0
        self.gamma_r = 0
        
        self.alpha_l = 0.75123
        self.beta_l = 0
        self.gamma_l = 0

        timer_period = 0.1  # seconds

        self.publisher_ = self.create_publisher(DutyCycles, '/motor/duty_cycles', 10)
        self.timer = self.create_timer(timer_period, self.move_callback)

        self.encoder = self.create_subscription(Encoders,'/motor/encoders',self.encoder_callback,10)
        self.encoder  # prevent unused variable warning

        self.desired = self.create_subscription(Twist,'/motor_controller/twist',self.desired_callback,10)
        self.desired  # prevent unused variable warning
        
    def move_callback(self):
        msg = DutyCycles()

        msg.duty_cycle_left = self.motion(self.alpha_l, self.beta_l, self.gamma_l, False)
        msg.duty_cycle_right = self.motion(self.alpha_r, self.beta_r, self.gamma_r) * self.correcting_term_r
        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: {} and {}'.format(msg.duty_cycle_left,msg.duty_cycle_right))
    
    def encoder_callback(self, msg):
        self.actual_wr = (2*math.pi*msg.delta_encoder_right *10)/360
        self.actual_wl = (2*math.pi*msg.delta_encoder_left *10)/360

    def desired_callback(self, msg):
        self.desired_v = msg.linear.x
        self.desired_w = msg.angular.z

        self.desired_wr = (self.desired_v - (self.base * self.desired_w))/self.radius
        self.desired_wl = (self.desired_v + (self.base * self.desired_w))/self.radius
    
    def motion(self, a, b, c, r=True):
        if r: #right motor
            error = self.desired_wr - self.actual_wr
            self.int_error_r = self.int_error_r + (error * self.dt)

            #NEED TO THRESHOLD INTEGRAL ERROR

            self.int_error_r = min(max(self.int_error_r,-110), 110)

            self.d_error_r = (error - self.prev_error_r)/self.dt
            self.prev_error_r = error

            pwm = (a * error) + (b * self.int_error_r) + (c * self.d_error_l)
            pwm = min(max(pwm,-100), 100)/100
            self.get_logger().info('error right: {} and int r:{}'.format(error,self.int_error_r))
        else: #left motor
            error = self.desired_wl - self.actual_wl
            self.int_error_l = self.int_error_l + (error * self.dt)

            #NEED TO THRESHOLD INTEGRAL ERROR
            self.int_error_l = min(max(self.int_error_l,-110), 110)
            
            self.d_error_l = (error - self.prev_error_l)/self.dt
            self.prev_error_l = error

            pwm = (a * error) + (b * self.int_error_l) + (c * self.d_error_l)
            pwm = min(max(pwm,-100), 100)/100
            self.get_logger().info('error left: {} and int l:{}'.format(error,self.int_error_l))
        return pwm
    


def main():
    rclpy.init()
    node = CartesianController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()