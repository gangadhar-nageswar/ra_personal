# #!/usr/bin/env python

# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import pyrealsense2 as rs 

# class ImageCaptureNode:
#     def __init__(self):
#         rospy.init_node('image_capture_node', anonymous=True)
#         self.publisher_pose2D = rospy.Publisher('/capture_image', Image, queue_size=10)
#         self.bridge = CvBridge()

#         pipeline = rs.pipeline()
#         config = rs.config()
#         config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#         # Start streaming
#         pipeline.start(config)
#         self.cap = pipeline.wait_for_frames()
        
#         # cv2.VideoCapture("/dev/video0")
#         # self.cap = cv2.VideoCapture("/home/share/audio2photoreal/saksham.mp4")

#         self.frame_count = 0
#         # print(self.frame_count)
#         self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)  # Adjust timer as needed

#     def timer_callback(self, event):
#         ret, frame = self.cap.read()
#         if not ret:
#             rospy.logwarn("No frame captured")
#             self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0) # For looping the playback video comment for the stream
#             return
#         try:
#             w, h = frame.shape[0], frame.shape[1]
#             print(w, h)
#             frame = frame[:,h//3:2*h//3, :]
#             ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
#         except CvBridgeError as e:
#             rospy.logwarn('Failed to convert frame to ROS image: %s' % e)
#             return

#         # Publish to pose2Dnode
#         self.publisher_pose2D.publish(ros_image)

#         # Publish every 10th frame to depth_estimation
#         # if self.frame_count % 10 == 0:
#         #     self.publisher_depth.publish(ros_image)

#         # self.frame_count += 1

# def main():
    
#     image_capture_node = ImageCaptureNode()
#     rospy.spin()

# if __name__ == '__main__':
#     main()
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs 
import numpy as np

class ImageCaptureNode:
    def __init__(self):
        rospy.init_node('image_capture_node', anonymous=True)
        self.publisher_capture_image = rospy.Publisher('/capture_image', Image, queue_size=10)
        self.bridge = CvBridge()

        # Initialize RealSense pipeline
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Set the color stream parameters
        pipeline.start(config)

        self.pipeline = pipeline

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)  # Adjust timer as needed

    def timer_callback(self, event):
        # Wait for the next available frame from the RealSense camera
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame:
            rospy.logwarn("No color frame captured")
            return

        # Convert the color frame to a numpy array
        frame_data = np.asanyarray(color_frame.get_data())
        # print(frame_data.shape)
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame_data, "bgr8")
        except CvBridgeError as e:
            rospy.logwarn('Failed to convert frame to ROS image: %s' % e)
            return

        # Publish the ROS image on the topic
        self.publisher_capture_image.publish(ros_image)

def main():
    image_capture_node = ImageCaptureNode()
    rospy.spin()

if __name__ == '__main__':
    main()
