#!/usr/bin/env python3
import can
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json

class PanelModule(Node):
    def __init__(self):
        super().__init__('panel_module')
        ##--------------------PANEL INDICATORS-----------------------##
        self.panel_module_id_tx = 0x410 #hex.410
        self.panel_module_id_rx = 0x409 #hex.409
        self.general_module_id_tx = 0x403
        self.general_module_id_rx = 0x404 
      
        # Provide the path to your JSON file
        file_path = '/home/ws/src/sdv_can/resources/panel_functionalities.json'
        
        # Read the JSON file and store its contents in a dictionary
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=125000)
        self.json_data = self.read_json_file(file_path)
        timer_period = 0.01 #Seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.object_key=""
        self.traffic_key=""
        self.lane_key=""
        self.safety_status = ""
        self.show_status = ""
        self.song_status = ""
        self.xbox_status={
            "giro_prominente_derecha":0,
            "giro_prominente_izquierda":0,
            "gran_trafico":0,
            "reset":0}
        self.battery_voltage = Float32()
        self.past_values = []
        self.category_accel = 0
        self.past_state_imu = 0
        self.audio_file = String()
        ##--------------------PANEL Publishers-----------------------##
        self.audio_panel= self.create_publisher(String, '/sdv/panel/speech', 10)
        self.pub_battery= self.create_publisher(Float32, '/sdv/panel/battery_voltage', 10)
      
        ##--------------------PANEL Subscribers-----------------------##

        self.object_sub = self.create_subscription(
            String,
            '/sdv/perception/object_status',
            self.object_notification_callback,
            10
        )
        self.traffic_sub = self.create_subscription(
            String,
            '/sdv/perception/traffic_status',
            self.recognize_traffic_callback,
            10
        )
        self.lane_sub = self.create_subscription(
            String,
            '/sdv/perception/lane_status',
            self.detect_lane_callback,
            10
        )

        self.safety_sub = self.create_subscription(
            String,
            '/sdv/panel/safety_mode',
            self.safety_mode_callback,
            10
        )

        self.show_sub = self.create_subscription(
            String,
            '/sdv/panel/show_mode',
            self.show_mode_callback,
            10
        )

        self.song_sub = self.create_subscription(
            String,
            '/sdv/panel/song_mode',
            self.song_mode_callback,
            10
        )
    ## AQUI VOY SHO###. ULISES s
        self.lidar_sub = self.create_subscription(
            int,
            '/warning_status', #ahorita buscar
            self.objectnotification2_callback,
        )
    ## AQUI TERMINO SHO ULISES ###
    
    def serializeFloatSingle(self, binNum: int):
        if not (binNum & ~(1<<31)):
            floatNum = 0
        else:
            sign = (binNum >> 31)
            expn = (binNum >> 23) & ((1<<8)-1)
            bias = (1<<(8-1))-1
            bexp = expn - bias
            frac = (binNum) & ((1<<23)-1)

            floatNum = (-1)**sign * (2**bexp) * (1 + frac/(1<<23))
            
        return float(floatNum)
    def read_json_file(self, file_path):
        with open(file_path, 'r') as file:
            data = json.load(file)
        return data
    def object_notification_callback(self, msg):
        self.object_key = msg.data
    def recognize_traffic_callback(self, msg):
        self.traffic_key = msg.data
    def detect_lane_callback(self, msg):
        self.lane_key = msg.data
    def km_to_m(self, km):
        return km*1000/pow(3600,2)
    def safety_mode_callback(self, msg):
        self.safety_status = msg.data
    def show_mode_callback(self, msg):
        self.show_status = msg.data
    def song_mode_callback(self, msg):
        self.song_status =  msg.data 
    def objectnotification2_callback(self,msg):
        self.objectnotification2_status = msg.data   
        ##msg.data nos va a arrojar un numero
        if msg.data == 1:
            msg.data = "far"
        elif msg.data == 2: 
            msg.data = "close"
        else:
            msg.data = "nothing"
    def timer_callback(self):
        try:
            
            # Object Notification agarrarlo de perception
            if(self.object_key!="" and self.object_key in self.json_data["objectNotification"]):
                data_can = self.json_data["objectNotification"][self.object_key][0]
                self.bus.send(can.Message(arbitration_id=self.panel_module_id_tx,is_extended_id=False, data=data_can),timeout=1)
                self.audio_file.data =  self.json_data["objectNotification"][self.object_key][1]
                if len(self.audio_file.data)>=1:
                    self.audio_panel.publish(self.audio_file)
            
            #AQUI VOY SHO DE NUEVO. ULISES  ###
             if(self.objectnotification2_status!="" and self.objectnotification2_status in self .json_data["objectNotification"]):
                data_can = self.json_data["objectNotification"][self.object_key][0]
                self.bus.send(can.Message(arbitration_id=self.panel_module_id_tx,is_extended_id=False, data=data_can),timeout=1)
                self.audio_file.data =  self.json_data["objectNotification"][self.object_key][1]
            ##CREO QUE AHI ESTA. NOT SURE. ULISLES

            ### AQUI TERMINO SHO
                             
            if(self.object_key!="" and self.object_key in self.json_data["ObjectNotification"]):
                data_can = self
            # Recognize traffic agarrarlo de perception
            if(self.traffic_key!="" and self.traffic_key in self.json_data["recognizeTraffic"]):
                data_can = self.json_data["recognizeTraffic"][self.traffic_key][0]
                self.bus.send(can.Message(arbitration_id=self.panel_module_id_tx,is_extended_id=False, data=data_can),timeout=1)
                self.audio_file.data =  self.json_data["recognizeTraffic"][self.traffic_key][1]
                if len(self.audio_file.data)>=1:
                    self.audio_panel.publish(self.audio_file)
            # Detect Lane agarrarlo de perception
            if(self.lane_key!="" and self.lane_key in self.json_data["detectLane"]):
                data_can = self.json_data["recognizeTraffic"][self.lane_key][0]
                self.bus.send(can.Message(arbitration_id=self.panel_module_id_tx,is_extended_id=False, data=data_can),timeout=1)
            
            # Safety mode 
            if(self.safety_status!="" and self.safety_status in self.json_data["safety_mode"]):
                data_can = self.json_data["safety_mode"][self.safety_status][0]
                self.bus.send(can.Message(arbitration_id=self.panel_module_id_tx,is_extended_id=False, data=data_can),timeout=1)
            
            # Show mode 
            if(self.show_status!="" and self.show_status in self.json_data["show_mode"]):
                data_can = self.json_data["show_mode"][self.show_status][0]
                self.bus.send(can.Message(arbitration_id=self.panel_module_id_tx,is_extended_id=False, data=data_can),timeout=1)
            
            # Song mode 
            if(self.song_status!="" and self.song_status in self.json_data["song_mode"]):
                self.audio_file.data =  self.json_data["song_audio_mode"][str(receivedMsg.data[1])]
                if len(self.audio_file.data)>=1:
                    self.audio_panel.publish(self.audio_file)
            # Publish battery voltage and general messages
            receivedMsg = self.bus.recv(1)
            if receivedMsg is not None:
                print(str(receivedMsg))
                if receivedMsg.arbitration_id == self.panel_module_id_rx:
                    if receivedMsg.data[0] == 0x5:
                        data_volts =  (receivedMsg.data[1] << 24) | (receivedMsg.data[2] << 16) | (receivedMsg.data[3] << 8) | (0 << 0)
                        self.battery_voltage.data = self.serializeFloatSingle(data_volts)
                        self.pub_battery.publish(self.battery_voltage)
                elif receivedMsg.arbitration_id == self.general_module_id_tx:
                    print(receivedMsg.data[0])
                    if receivedMsg.data[0] == 0x0:
                        self.audio_file.data =  self.json_data["emergency_audio_mode"][str(receivedMsg.data[1])]
                        if len(self.audio_file.data)>=1:
                            self.audio_panel.publish(self.audio_file)
                    elif receivedMsg.data[0] == 0x2:
                        self.audio_file.data =  self.json_data["drive_audio_mode"][str(receivedMsg.data[1])]
                        if len(self.audio_file.data)>=1:
                            self.audio_panel.publish(self.audio_file)
                    elif receivedMsg.data[0] == 0x3:
                        self.audio_file.data =  self.json_data["driver_present_audio_mode"][str(receivedMsg.data[1])]
                        if len(self.audio_file.data)>=1:
                            self.audio_panel.publish(self.audio_file)
      
        except Exception as error:
            self.get_logger().debug('Panel has failed')
    

def main(args=None):
    rclpy.init(args=args)

    panel = PanelModule()
    panel.get_logger().info('Panel node started')
    rclpy.spin(panel)
    panel.destroy_node()
    rclpy.shutdown()
            
if __name__ == "__main__":
    
    main()