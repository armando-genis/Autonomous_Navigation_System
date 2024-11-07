#!/usr/bin/env python3

import can
import can.interfaces.socketcan as socketcan
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int64
import math

class RM8004Encoder(Node, can.Listener):
    def __init__(self):
        super().__init__('encoder_rm8004')

        self.declare_parameter('channel', rclpy.Parameter.Type.STRING)
        self.declare_parameter('bitrate', rclpy.Parameter.Type.INTEGER)
        channel = self.get_parameter('channel').value
        bitrate = self.get_parameter('bitrate').value

        self.get_logger().info(f"Using interface: {channel}, bitrate: {bitrate}")

        # Publishers
        self.steering_angle_pub = self.create_publisher(Float64, '/sdv/state/steering/angle', 10)
        self.steering_raw_pub = self.create_publisher(Int64, '/sdv/state/steering/raw', 10)

        self.encoder_steps_per_rev = 4096.0

        filters = [
            {"can_id": 0x1A0, "can_mask": 0x1A0, "extended": False}
        ]
        self.bus = can.interface.Bus(bustype='socketcan', channel=channel, bitrate=bitrate, can_filters=filters)

        self.notifier = can.Notifier(self.bus, [self])
    
    def on_message_received(self, msg) -> None:
        if msg is None:
            return
        
        if msg.arbitration_id == 0x1A0:
            if msg.dlc != 4:
                raise Exception(f"Received message for IFM encoder, but length {msg.dlc} is not valid")
            raw_position = int.from_bytes(msg.data[0:4], biteorder='little', signed=False)
            raw_position_msg = Int64()
            raw_position_msg.data = raw_position
            self.steering_raw_pub.publish(raw_position_msg)

            # Convert to rads
            position = (float(raw_position) / self.encoder_steps_per_rev) * math.pi * 2
            position_msg = Float64()
            position_msg.data = position
            self.steering_angle_pub.publish(position_msg)

def main(args=None):
    rclpy.init(args=args)
    encoder_publisher = RM8004Encoder()
    rclpy.spin(encoder_publisher)
    encoder_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
