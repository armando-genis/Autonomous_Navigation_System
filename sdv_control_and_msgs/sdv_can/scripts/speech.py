#!/usr/bin/env python3

import rclpy
import subprocess
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

class Speech(Node):

    def __init__(self, dev_name):
        super().__init__("speech")
        self.get_logger().info("Speech Node Started")

        # Name and address of the bluetooth device
        self.device_name = dev_name
        self.device_address = None
        self.speech_connected = False

        # List of requested sounds
        self.speech_queue = []

        # Last requested speech
        self.last_speech = None
        self.bluetooth_device = False
        
        # Dictionary of possible messages
        self.sounds = {"modo_autonomo" : "/sounds/autonomous_mode.wav", 
                       "modo_manual" : "/sounds/manual_mode.wav",
                        "joke" : "/sounds/joke.wav",
                        "distant_object" : "/sounds/distant_object.wav",
                        "nearby_object" : "/sounds/nearby_object.wav",
                        "stop_signal" : "/sounds/stop_signal.wav",
                        "pedestrian_crossing" : "/sounds/pedestrian_crossing.wav",
                        "pedestrian_signal" : "/sounds/pedestrian_signal.wav",
                        "pedestrian_around" : "/sounds/pedestrian_around.wav",
                        "on" : "/sounds/on.wav",
                        "off" : "/sounds/off.wav",   
                        "good_morning_sir": "/sounds/good_morning_sir.wav",  
                        "song_tec" : "/sounds/song_tec.wav",   
                       }

        # Get the package path
        package_name = "sdv_can"
        self.package_path = get_package_share_directory(package_name).replace("/share/" + package_name, "").replace("install", "src")

        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.speech_name_subscription = self.create_subscription(String, '/sdv/panel/speech', self.speech_name_callback, 10)


    def speech_name_callback(self, msg):
        if msg.data != self.last_speech:
            self.last_speech = msg.data
            self.speech_queue.append(msg.data)


    def timer_callback(self):
        # Check device bluetooth state and connect
        try:
            if self.bluetooth_device  == True:
                self.get_logger().info(f"Trying to connect to: {self.device_name}")
                if not self.check_if_paired(self.device_name):
                    if self.bluetooth_device_pair_connect(self.device_name):
                        self.get_logger().info(f"Successfully paired and connected to {self.device_name}")
                        self.speech_connected = True
                    else:
                        self.get_logger().info(f"Could not find {self.device_name} in nearby devices")
                        self.speech_connected = False
                else:
                    if not self.check_if_connected(self.device_address):
                        if not self.bluetooth_device_connect(self.device_address):
                            self.get_logger().info(f"Successfully connected to {self.device_name}")
                            self.speech_connected = True
                        else:
                            self.get_logger().info(f"Could not connect to {self.device_name}")
                            self.speech_connected = False
                    else:
                        self.get_logger().info(f"Already connected to {self.device_name}")
                        self.speech_connected = True

            
            if  self.bluetooth_device ==False or (self.speech_connected and len(self.speech_queue)) :
                print(self.speech_queue)
                self.play_speech(self.package_path + self.sounds[self.speech_queue.pop(0)])
        except Exception as error:
            self.get_logger().info('Speech has failed'+str(error))
    def check_if_paired(self, device_name):
        try:
            devices = subprocess.check_output(['bluetoothctl', 'paired-devices']).decode('utf-8').splitlines()

            for device in devices:
                if device_name in device:
                    self.device_address = device.split()[1]
                    return True
            return False
        except subprocess.CalledProcessError as e:
            return False

        
    def bluetooth_device_pair_connect(self, device_name):
        # Get a list of nearby Bluetooth devices
        try:
            # devices  = subprocess.check_output(['hcitool', 'scan']).decode('utf-8').splitlines()
            # for device in devices:
            #     # Find the device address based on its name
            #     if device_name in device:
            #         self.device_address = device.split()[0]
            #         self.get_logger().info(f"euuu to {device}")
            # if self.device_address:
                # Pair and connect to the Bluetooth device
            self.device_address = device_name
            subprocess.run(['bluetoothctl', 'trust', self.device_address], stdout=subprocess.DEVNULL)
            
            subprocess.run(['bluetoothctl', 'connect', self.device_address], stdout=subprocess.DEVNULL)

            return True
        except subprocess.CalledProcessError as e:
            return False



    
    def check_if_connected(self, device_address):
        info = subprocess.check_output(['bluetoothctl', 'info', device_address]).decode('utf-8').splitlines()

        for i in info:
            if i.split()[0] == 'Connected:' and i.split()[1] == 'yes':
                return True
        return False

    def bluetooth_device_connect(self, device_address):
        return not subprocess.run(['bluetoothctl', 'connect', device_address], stdout=subprocess.DEVNULL).returncode

    def play_speech(self, file_path):
        # subprocess.run(['aplay', '-D', 'bluealsa', file_path])
        # Use aplay to play speech through the speech output
        subprocess.run(['aplay', file_path], stdout=subprocess.DEVNULL)
        
# sed -i "s/; enable-shm = yes/enable-shm = no/g" /etc/pulse/daemon.conf
# sed -i "s/; enable-shm = yes/enable-shm = no/g" /etc/pulse/client.conf
def main(args=None):
    rclpy.init(args=args)
    
    #node = Speech('JAM Rave Plus')
    #node = Speech('WH-CH710')
    node = Speech('C8:7B:23:95:25:8A')
    rclpy.spin(node)
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()