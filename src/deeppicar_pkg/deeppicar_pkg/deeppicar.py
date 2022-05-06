import os
import json
import rclpy
from deeppicar_pkg import input_kbd
from rclpy.node import Node

from deepracer_interfaces_pkg.msg import (ServoCtrlMsg, CameraMsg)

#Constants
DEEPPICAR_DRIVE_TOPIC_NAME = "deeppicar_pkg/servo_msg"

#Globals
frame_id = 0

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
        # TODO: Add recorder callback function
        '''self.deeppicar_camera_subscriber = self.create_subscription( CameraMsg,
                                                                     "/camera_pkg/video_mjpeg",
                                                                     self.recorder,
                                                                     1)
        '''
        timer_period = 0.05 #seconds
        self.timer = self.create_timer(timer_period, self.control)


    def control(self):
        ch = ord(input_kbd.read_single_keypress())
        change = True
        servo = ServoCtrlMsg()
        if ch != 32:
            if ch == ord('j'): # left
                self.angle += 0.1
                self.angle = min(0.8, self.angle)
                servo.angle = self.angle
                print ("\nleft")
            elif ch == ord('k'): # center
                self.angle = 0.0
                servo.angle = self.angle
                print ("\ncenter")
            elif ch == ord('l'): # right
                self.angle -= 0.1
                self.angle = max(-0.8, self.angle)
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

    # TODO: Add recorder callback function
    '''def recorder(self, msg):
        if enable_record == True and frame_id == 0:
            # create files for data recording
            keyfile = open(params.rec_csv_file, 'w+')
            keyfile.write("ts_micro,frame,wheel\n")
            try:
                fourcc = cv2.cv.CV_FOURCC(*'XVID')
            except AttributeError as e:
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
            vidfile = cv2.VideoWriter(params.rec_vid_file, fourcc,
                                    cfg_cam_fps, cfg_cam_res)
            if enable_record == True and frame is not None:
                # increase frame_id
                frame_id += 1

                # write input (angle)
                str = "{},{},{}\n".format(int(ts*1000), frame_id, self.angle)
                keyfile.write(str)
            
            # write video stream
            vidfile.write(frame)
            #img_name = "cal_images/opencv_frame_{}.png".format(frame_id)
            #cv2.imwrite(img_name, frame)
            if frame_id >= 1000:
                print ("recorded 1000 frames")
                break
            print ("%.3f %d %.3f %d(ms)" %
               (ts, frame_id, angle, int((time.time() - ts)*1000)))'''

def main(args=None):
    rclpy.init(args=args)
    deeppicar_node = DeepPicarNode()
    rclpy.spin(deeppicar_node)

    deeppicar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
