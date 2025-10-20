# !/usr/bin/env/ python3
#
# Copyright 2023 EduRobotAILab CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Leo Cho

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraPub(Node):

    def __init__(self):
        super().__init__('camera_pub')       
        
        self.pub = self.create_publisher(Image, 'camera', 10)      
        timer_period = 0.1        
        self.timer = self.create_timer(timer_period, self.timer_callback)        
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,640) 
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)         
        self.bridge = CvBridge()

    def timer_callback(self):
      
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("Not Found Devices")
        frame = cv2.flip(frame,0)
        frame = cv2.flip(frame,1)        
        self.pub.publish(self.bridge.cv2_to_imgmsg(frame))
            
def main(args=None):
    rclpy.init(args=args)

    camera_pub = CameraPub()

    try:
        rclpy.spin(camera_pub)
    except KeyboardInterrupt:
        camera_pub.get_logger().info('keyboard Interrupt')
    except Exception as exception_error:
        camera_pub.get_logger().info("Error: " + str(exception_error))
    finally:
        camera_pub.cap.release()
        camera_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
