
import rclpy
from rclpy.node import Node

import numpy as np

from copy import copy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo

from openni import openni2
from openni import _openni2 as c_api

from cv_bridge import CvBridge


class ArucoNode(Node):
    def __init__(self):
        super().__init__('Depth_Node')
        self.depth_pub = self.create_publisher(Image, '/astra_camera/depth', 10)
        self.image_pub = self.create_publisher(Image, '/astra_camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/astra_camera/camera_info', 10)

        timer_period = 0.01  # seconds
        info_timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.info_timer = self.create_timer(info_timer_period, self.info_timer_callback)

    def info_timer_callback(self):
        camera_info = CameraInfo()
        camera_info.height = 480
        camera_info.width = 640
        camera_info.distortion_model = "plumb_bob"
        camera_info.header.frame_id = "astra_camera"
        camera_info.header.stamp = self.get_clock().now().to_msg()
        self.info_pub.publish(camera_info)

    def timer_callback(self):
        # Initialize the depth device
        openni2.initialize()
        dev = openni2.Device.open_any()

        # Start color stream
        colour_stream = dev.create_color_stream()
        colour_stream.start()
        colour_stream.set_video_mode(c_api.OniVideoMode(pixelFormat = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_RGB888, resolutionX = 640, resolutionY = 480, fps = 30)) 

        # Start the depth stream
        depth_stream = dev.create_depth_stream()
        depth_stream.start()
        depth_stream.set_video_mode(c_api.OniVideoMode(pixelFormat = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_100_UM, resolutionX = 640, resolutionY = 480, fps = 30))

        while rclpy.ok():
            color = colour_stream.read_frame()
            color_data = color.get_buffer_as_uint8()
            image = np.frombuffer(color_data, dtype=np.uint8)
            image = image.reshape(480, 640, 3)
            image[:, :, [0,2]] = image[:, :, [2,0]]

            frame = depth_stream.read_frame()
            frame_data = frame.get_buffer_as_uint16()
            depth_img = np.frombuffer(frame_data, dtype=np.uint16)
            depth_img = depth_img.reshape(480, 640)

            image_result = Image()
            bridge = CvBridge()
            image_result = bridge.cv2_to_imgmsg(image, encoding="passthrough")
            self.image_pub.publish(image_result)

            depth_result = Image()
            bridge2 = CvBridge()
            depth_result = bridge2.cv2_to_imgmsg(depth_img, encoding="passthrough")
            self.depth_pub.publish(depth_result)


def main(args=None):
    rclpy.init(args=args)
    Aruco_Node = ArucoNode()
    rclpy.spin(Aruco_Node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()