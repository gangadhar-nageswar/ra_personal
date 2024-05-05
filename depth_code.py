import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs

class RealSenseNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_intrinsics = None
        self.depth_scale = 0.001  # Default depth scale value (1 millimeter per unit)
        self.color_image = None
        self.depth_image = None
        self.camera_info_received = {'color': False, 'depth': False}

        # Subscribe to color and depth image topics
        self.color_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback)
        self.depth_subscriber = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        self.depth_camera_info_subscriber = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.depth_camera_info_callback)

        rospy.spin()

    def color_callback(self, color_image):
        self.color_image = self.bridge.imgmsg_to_cv2(color_image, desired_encoding="bgr8")

    def depth_callback(self, depth_image):
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")

    def depth_camera_info_callback(self, camera_info):
        # print(f"depthvsofjs: {camera_info}")
        # self.depth_intrinsics = camera_info
        self.depth_intrinsics = rs.pyrealsense2.intrinsics()
        self.depth_intrinsics.height = camera_info.height
        self.depth_intrinsics.width = camera_info.width
        self.depth_intrinsics.ppx = 213.47621154785156
        self.depth_intrinsics.ppy = 121.29695892333984
        self.depth_intrinsics.fx = 306.0126953125
        self.depth_intrinsics.fy = 306.1602783203125
        self.depth_intrinsics.model = rs.pyrealsense2.distortion.inverse_brown_conrady
        self.depth_intrinsics.coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_received['depth'] = True

        # Check if both color and depth camera info are received
        if self.camera_info_received['color'] or self.camera_info_received['depth']:
            self.process_depth_at_point(200, 200)  # Example point

    def process_depth_at_point(self, x, y):
        # Check if depth image and camera intrinsics are available
        if self.depth_image is not None and self.depth_intrinsics is not None:
            depth_meters, point = self.get_depth_at_point(x, y)
            print(f"Depth at point ({x}, {y}): {depth_meters} meters")
            print(f"Real-world coordinates at point ({x}, {y}): {point}")
        else:
            rospy.logwarn("Depth image or camera intrinsics not available yet.")

    def get_depth_at_point(self, x, y):
        # Get depth value at point (x, y)
        depth_value = self.depth_image[y, x]

        # Calculate real-world coordinates
        depth_in_meters = depth_value * self.depth_scale
        pixel = [float(x), float(y)]  # Convert pixel coordinates to floats
        point = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, pixel, depth_in_meters)

        return depth_in_meters, point

if __name__ == "__main__":
    rospy.init_node('realsense_node', anonymous=True)
    RealSenseNode()
