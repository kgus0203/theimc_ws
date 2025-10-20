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

class CameraSub(Node):

    def __init__(self):
        super().__init__('camera_sub')       

        self.sub = self.create_subscription(
            Image, 
            'camera', 
            self.listener_callback, 
            10)
        self.sub
        self.bridge = CvBridge()

    def listener_callback(self, data):
        
        frame_sub = self.bridge.imgmsg_to_cv2(data)       
        cv2.imshow("camera", frame_sub)
        cv2.waitKey(1)
            
def main(args=None):
    rclpy.init(args=args)

    camera_sub = CameraSub()

    try:
        rclpy.spin(camera_sub)
    except KeyboardInterrupt:
        camera_sub.get_logger().info('keyboard Interrupt')
    except Exception as exception_error:
        camera_sub.get_logger().info("Error: " + str(exception_error))
    finally:
        camera_sub.cap.release()
        camera_sub.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
