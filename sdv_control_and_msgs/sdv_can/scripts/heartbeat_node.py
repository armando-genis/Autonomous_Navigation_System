#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
import can
from  std_msgs.msg import UInt8MultiArray
class Heartbeat(Node):
    def __init__(self):
        super().__init__('heartbit_node_started')      
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=125000)
        self.timeout = 3
        self.module_pub = self.create_publisher(UInt8MultiArray,"/sdv/diagnostics/module_status", 10)
        self.module_msg = UInt8MultiArray()

        # *------------------* VANTTEC_IDS-TX *------------------*
        #Dict -> key: Module, value:[name,bool if module has died, actual time , prev time]
        self.module_status = {
           0x403: ["general",False,0,0],
           0x405: ["throttle", False,0,0],
           0x407: ["panel", False,0,0],
           0x409: ["steering", False,0,0],
           0x411: ["brake", False,0,0]
        }

    def timer_callback(self):
        #Analyse if any module stopped transmitting heartbeat counter
        msg =  self.bus.recv(1)
        if msg is not None:
            if msg.arbitration_id in self.module_status:  
                self.module_status[msg.arbitration_id][2] = self.get_clock().now().to_msg().sec
                if self.module_status[msg.arbitration_id][2] - self.module_status[msg.arbitration_id][3] >= self.timeout:
                    #Module not activated
                    self.module_status[msg.arbitration_id][1] = False
                else:
                    #Module activated
                    self.module_status[msg.arbitration_id][1] = True
                self.module_status[msg.arbitration_id][3] = self.module_status[msg.arbitration_id][2]
                self.module_msg.data = [int(value[1]) for value in self.module_status.values()]
                self.module_pub.publish(self.module_msg)

def main(args=None):
    rclpy.init(args=args)

    heartbeat = Heartbeat()
    heartbeat.get_logger().info('Heartbeat node started')
    rclpy.spin(heartbeat)
    heartbeat.destroy_node()
    rclpy.shutdown()
            
if __name__ == "__main__":
    
    main()
