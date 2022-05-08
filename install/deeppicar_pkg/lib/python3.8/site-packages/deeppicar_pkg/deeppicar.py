import os
import json
import rclpy
import numpy as np
import cv2
from cv_bridge import CvBridge
from time import time
from math import pi
from deeppicar_pkg import input_kbd
from deeppicar_pkg import params
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from deepracer_interfaces_pkg.msg import (ServoCtrlMsg, CameraMsg)

print ("Loading model: " + params.model_file)
try:
    # Import TFLite interpreter from tflite_runtime package if it's available.
    from tflite_runtime.interpreter import Interpreter
    interpreter = Interpreter(params.model_file+'.tflite', num_threads=2)
except ImportError:
    # If not, fallback to use the TFLite interpreter from the full TF package.
    import tensorflow as tf
    interpreter = tf.lite.Interpreter(model_path=params.model_file+'.tflite', num_threads=2)

class DeepPicarNode(Node):
    #Constants
    DEEPPICAR_DRIVE_TOPIC_NAME = "deeppicar_pkg/servo_msg"
    CAMERA_VIDEO_TOPIC_NAME = "/camera_pkg/video_mjpeg"
    CATEGORY_VALUES = [-0.85, -0.55, -0.35, 0.0, 0.35, 0.55, 0.85]
    cfg_cam_res = (640, 480)
    cfg_cam_fps = 30
    bridge = CvBridge()
    
    #Local Variables
    throttle = 3
    use_dnn = False
    angle = 3
    keyfile = None
    vidfile = None
    frame_id = 0
    enable_record = False

    def __init__(self):
        super().__init__('deeppicar_node')
        self.get_logger().info("deeppicar_node started")

        # Taken from deepracer_navigation_pkg
        self.deeppicar_drive_publisher = self.create_publisher( ServoCtrlMsg, 
                                                                self.DEEPPICAR_DRIVE_TOPIC_NAME,
                                                                1)
        
        # Camera function is called when a new camera message appears
        self.deeppicar_camera_subscriber = self.create_subscription( CameraMsg,
                                                                     self.CAMERA_VIDEO_TOPIC_NAME,
                                                                     self.camera_funcs,
                                                                     1)
        
        timer_period = 0.1 #seconds
        self.timer = self.create_timer(timer_period, self.control)

    def turn_off(self):
        # Center steering and turn off motor
        servo = ServoCtrlMsg()
        servo.angle = 0.0
        servo.throttle = 0.0
        self.deeppicar_drive_publisher.publish(servo)
        # If recording occurred close out files
        if self.frame_id > 0:
            self.keyfile.close()
            self.vidfile.release()
    
    def rad2deg(self,rad):
        return 180.0 * rad / pi

    def camera_funcs(self, camera_msg):
        ts = time()
        # Converting ROS image format to OpenCV UMat
        frame = self.bridge.imgmsg_to_cv2(camera_msg.images[0], desired_encoding='passthrough')
        if self.use_dnn:
            self.dnn(frame)
        if self.enable_record:
            self.recorder(frame, ts)

    def preprocess(self, img):
        img = cv2.resize(img, (params.img_width, params.img_height))
        # Convert to grayscale and readd channel dimension
        if params.img_channels == 1:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img = np.reshape(img, (params.img_height, params.img_width, params.img_channels))
        img = img / 255.
        return img

    def control(self):
        ch = ord(input_kbd.read_single_keypress())
        change = True
        servo = ServoCtrlMsg()
        if ch != 32:
            if ch == ord('j'): # left
                self.angle = min(6, self.angle+1)
                print ("\nleft")
            elif ch == ord('k'): # center
                self.angle = 3
                print ("\ncenter")
            elif ch == ord('l'): # right
                self.angle = max(0, self.angle-1)
                print ("\nright")
            elif ch == ord('a'):
                self.throttle = min(6, self.throttle+1)
                print ("\naccel")
            elif ch == ord('s'):
                self.throttle = 3
                print ("\nstop")
            elif ch == ord('z'):
                self.throttle = max(0, self.throttle-1)
                print ("\nreverse")
            elif ch == ord('r'):
                change = False
                if self.enable_record:
                    print ("\ndisable record mode")
                else:
                    print("\nenable record mode")
                self.enable_record = not self.enable_record
            elif ch == ord('d'):
                change = False
                print ("\ntoggle DNN mode")
                self.use_dnn = not self.use_dnn
            else:
                change = False
            if change:
                servo.throttle = self.CATEGORY_VALUES[self.throttle]
                servo.angle = self.CATEGORY_VALUES[self.angle]
                print('Throttle:' + str(servo.throttle))
                print('Angle:' + str(servo.angle))
            self.deeppicar_drive_publisher.publish(servo)
    
    def dnn(self, frame):
        img = self.preprocess(frame)
        img = np.expand_dims(img, axis=0).astype(np.float32)
        interpreter.set_tensor(input_index, img)
        interpreter.invoke()
        angle = interpreter.get_tensor(output_index)[0][0]
        degree = self.rad2deg(angle)
        if degree <= -15:
            servo.angle = self.CATEGORY_VALUES[1]
            self.angle = 1
            print ("left (CPU)")
        elif degree < 15 and degree > -15:
            servo.angle = self.CATEGORY_VALUES[2]
            self.angle = 2
            print ("center (CPU)")
        elif degree >= 15:
            servo.angle = self.CATEGORY_VALUES[3]
            self.angle = 3
            print ("right (CPU)")
        servo.throttle = self.CATEGORY_VALUES[3]
        self.deeppicar_drive_publisher.publish(servo)

    def recorder(self, frame, ts):
        if self.frame_id == 0:
            # create files for data recording
            self.keyfile = open(params.rec_csv_file, 'w+')
            self.keyfile.write("ts_micro,frame,wheel\n")
            try:
                fourcc = cv2.cv.CV_FOURCC(*'XVID')
            except AttributeError as e:
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.vidfile = cv2.VideoWriter(params.rec_vid_file, fourcc,
                                    self.cfg_cam_fps, self.cfg_cam_res)
        if frame is not None:
            # increase frame_id
            self.frame_id += 1

            # write input (angle)
            str = "{},{},{}\n".format(int(ts*1000), self.frame_id, self.CATEGORY_VALUES[self.angle])
            self.keyfile.write(str)
       
            # write video stream
            self.vidfile.write(frame)
            if self.frame_id >= 1000:
                print ("recorded 1000 frames")
                self.enable_record = False
                self.turn_off()
            print ("%.3f %d %.3f %d(ms)" %
               (ts, self.frame_id, self.CATEGORY_VALUES[self.angle], int((time() - ts)*1000)))

def main(args=None):
    rclpy.init(args=args)
    deeppicar_node = DeepPicarNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(deeppicar_node, executor)

    deeppicar_node.turn_off()
    deeppicar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
