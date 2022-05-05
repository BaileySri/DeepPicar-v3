import os
import json
import rclpy
from deeppicar_pkg import input_kbd
from rclpy.node import Node

from deepracer_interfaces_pkg.msg import (ServoCtrlMsg)

#Constants
DEEPPICAR_DRIVE_TOPIC_NAME = "deeppicar_drive"

#Local Variables
angle = 0
throttle = 0
use_dnn = False
class DeepPicarNode(Node):

    def __init__(self):
        super().__init__('deeppicar_node')
        self.get_logger().info("deeppicar_node started")
        # Taken from deepracer_navigation_pkg
        auto_drive_pub_msg_cb_group = ReentrantCallbackGroup()
        self.deeppicar_drive_publisher = self.create_publisher( ServoCtrlMsg, 
                                                                DEEPPICAR_DRIVE_TOPIC_NAME,
                                                                1)

    def control(self, key):
        servo = ServoCtrlMsg()
        if ch == ord('j'): # left
            angle -= 0.1
            angle = max(-1, angle)
            servo.angle = angle
            print ("left")
        elif ch == ord('k'): # center
            angle = 0
            servo.angle = angle
            print ("center")
        elif ch == ord('l'): # right
            angle += 0.1
            angle = min(1, angle)
            servo.angle = angle
            print ("right")
        elif ch == ord('a'):
            throttle -= 0.1
            throttle = min(1, throttle)
            servo.throttle = throttle
            print ("accel")
        elif ch == ord('s'):
            throttle = 0
            servo.throttle = 0
            print ("stop")
        elif ch == ord('z'):
            throttle += 0.1
            throttle = max(-1, throttle)
            servo.throttle = throttle
            print ("reverse")
        elif ch == ord('r'):
            print ("toggle record mode")
            enable_record = not enable_record
        elif ch == ord('d'):
            print ("toggle DNN mode")
            use_dnn = not use_dnn
        self.deeppicar_drive_publisher.publish(servo)

def main(args=None):
    rclpy.init(args=args)
    deeppicar_node = DeepPicarNode()
    
    try:
        while True:
            print("Input Control Character:"),
            key = ord(input_kbd.read_single_keypress())
            #control(key)
            print(key)
    except Exception as e:
        print(e)

    deeppicar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
