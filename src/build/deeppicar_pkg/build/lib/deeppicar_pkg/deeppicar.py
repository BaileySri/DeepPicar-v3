import os
import json
import rclpy
from deeppicar_pkg import input_kbd
from rclpy.node import Node

from deepracer_interfaces_pkg.msg import (ServoCtrlMsg)

#Constants
DEEPPICAR_DRIVE_TOPIC_NAME = "deeppicar_drive"


class DeepPicarNode(Node):
    #Local Variables
    throttle = 0.0
    use_dnn = False
    angle = 0.0
    
    def __init__(self):
        super().__init__('deeppicar_node')
        self.get_logger().info("deeppicar_node started")
        # Taken from deepracer_navigation_pkg
        self.deeppicar_drive_publisher = self.create_publisher( ServoCtrlMsg, 
                                                                DEEPPICAR_DRIVE_TOPIC_NAME,
                                                                1)
        timer_period = 0.05 #seconds
        self.timer = self.create_timer(timer_period, self.control)


    def control(self):
        ch = ord(input_kbd.read_single_keypress())
        change = True
        servo = ServoCtrlMsg()
        if ch == ord('j'): # left
            self.angle -= 0.1
            self.angle = max(-1.0, self.angle)
            servo.angle = self.angle
            print ("\nleft")
        elif ch == ord('k'): # center
            self.angle = 0.0
            servo.angle = self.angle
            print ("\ncenter")
        elif ch == ord('l'): # right
            self.angle += 0.1
            self.angle = min(1.0, self.angle)
            servo.angle = self.angle
            print ("\nright")
        elif ch == ord('a'):
            self.throttle -= 0.1
            self.throttle = min(1.0, self.throttle)
            servo.throttle = self.throttle
            print ("\naccel")
        elif ch == ord('s'):
            self.throttle = 0.0
            servo.throttle = 0.0
            print ("\nstop")
        elif ch == ord('z'):
            self.throttle += 0.1
            self.throttle = max(-1.0, self.throttle)
            servo.throttle = self.throttle
            print ("\nreverse")
        elif ch == ord('r'):
            change = False
            print ("\ntoggle record mode")
            enable_record = not enable_record
        elif ch == ord('d'):
            change = False
            print ("\ntoggle DNN mode")
            self.use_dnn = not self.use_dnn
        else:
            change = False
        if change:
            print('Throttle:' + str(self.throttle))
            print('Angle:' + str(self.angle))
        self.deeppicar_drive_publisher.publish(servo)

def main(args=None):
    rclpy.init(args=args)
    deeppicar_node = DeepPicarNode()
    rclpy.spin(deeppicar_node)

    deeppicar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
