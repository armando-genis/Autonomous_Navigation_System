#!/usr/bin/env python3

# do not launch this node alone unless you know what you are doing
# you are better of launching 'remote_steer.launch.py'

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Bool
from sdv_msgs.srv import Uint8

import math

class RemoteMapping(Node):

    def __init__(self):
        super().__init__('remote_joystick_mapping_node')
        self.jsub = self.create_subscription(
            Joy, '/joy', self.convert, 10
        )
        self.jsub

        # no me gustan los yams la verdad jajas huh?
        self.setpoint_pub = self.create_publisher(Float64, "/sdv/steering/setpoint", 10)
        self.setpoint_pub
        self.setpoint_msg = Float64()

        self.set_mode_srv = self.create_client(Uint8, '/sdv/steering/set_mode')
        self.set_mode_srv
        self.mode_req = Uint8.Request()
        self.mode_req.data = 1 # manual mode on default

        # self.zero_encoder_pub_ = self.create_publisher(Bool, "/sdv/steering/reset_encoder", 10)
        # self.zero_encoder_pub_
        # self.zero_encoder_msg = Bool()

        self.angle_increment = 0.15
        self.curr_angle = 0.0
        self.past_angle = 0.0

        self.dropout = 0.01

        self.reset_pressed_count = 0.0


    def convert(self, msg):
        #TODO modify dead zones??

        #[DEBUG]
        #msg.axes - son los valores de los joysticks
        #msg.buttons - son los valores de los botones

        joystick_axes_index = 0
        # reset_button_index = 6
        mode_button_index = 6  # TODO

        # -- setpoint -- #
        #[DEBUG] le ponemos un `-` para que la izquierda sea negativo y viceversa
        delta = -msg.axes[joystick_axes_index]
        self.curr_angle += delta * self.angle_increment
        self.curr_angle = max(-3 * math.pi, min(self.curr_angle, 3 * math.pi))

        if abs(self.curr_angle - self.past_angle) < self.dropout:
            self.curr_angle = self.past_angle
        self.setpoint_msg.data = self.curr_angle

        self.get_logger().info('joystick: "%f"' % self.curr_angle)
        self.setpoint_pub.publish(self.setpoint_msg)

        self.past_angle = self.curr_angle

        # -- mode -- #
        if msg.buttons[mode_button_index]:
            # [DEBUG] dumb way to toggle between 0 & 1
            self.mode_req.data = 1 - self.mode_req.data

            future = self.set_mode_srv.call_async(self.mode_req)
            future.add_done_callback(self.set_mode_done)

        # -- encoder zero -- #
        # if msg.buttons[reset_button_index]:
        #     self.reset_pressed_count += 1
        # else:
        #     self.reset_pressed_count = 0
        
        # self.zero_encoder_msg.data = False
        # if self.reset_pressed_count >= 10:
        #     self.zero_encoder_msg.data = True
        #     self.get_logger().warning('Resetting Zero in Steering Wheel')
            
    def set_mode_done(self):
        self.get_logger().info('changed mode!')


def main(args=None):
    rclpy.init(args=args)

    rmn = RemoteMapping()

    rclpy.spin(rmn)

    rmn.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
