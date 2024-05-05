#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs 
import numpy as np
import cv2
import os
import sys
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Point
from mrcnn import utils
import mrcnn.model as modellib
import book  # Import BOOK config

# Ignore depreciation warnings
import tensorflow as tf
tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)

class BookDetector:
    def __init__(self, root_dir="./Mask_RCNN", model_path="models/mask_rcnn_book_0999.h5"):
        # Root directory of the project
        ROOT_DIR = os.path.abspath(root_dir)

        # Import Mask RCNN
        sys.path.append(ROOT_DIR)  # To find local version of the library

        # Local path to trained weights file
        BOOK_MODEL_PATH = "./models/mask_rcnn_book_0999.h5"
        
        # Directory to save logs and trained model
        MODEL_DIR = os.path.join(ROOT_DIR, "logs")
        
        # Load config
        config = book.BookConfig()

        # Destination directory
        self.dest_dir = "/home/student/16662_RobotAutonomy/src/devel_packages/team8B/Library_Book_Detection/result"

        # Create model object in inference mode.
        self.model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=config)

        # Load weights trained on MS-COCO
        self.model.load_weights(BOOK_MODEL_PATH, by_name=True)

        # Initialize CvBridge
        self.bridge = CvBridge()

    # Check presence of detections
    def num_instances_check(self, results):
        # Number of instances
        r = results
        boxes = r['rois']
        masks = r['masks']
        class_ids = r['class_ids']
        N = boxes.shape[0]
        if not N:
            print("\n*** No instances to display *** \n")
            exit()
        else:
            assert boxes.shape[0] == masks.shape[-1] == class_ids.shape[0]
        print('{} instances detected'.format(N))
        return N
    
    # Model forward pass
    def detect(self, image):
        # Run detection
        results = self.model.detect([image], verbose=1)
        N = self.num_instances_check(results[0])

        return results, N

    # Compute the orientation of the bounding box
    def compute_orientation(self, bbox):
        """
        Compute the orientation (angle) of the bounding box
        :param bbox: Tuple containing (y1, x1, y2, x2) coordinates of the bounding box
        :return: Orientation angle in degrees
        """
        y1, x1, y2, x2 = bbox
        angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
        return angle

    # Segment detections, show, and save if required
    def segment(self, image, results, all_at_once=True, show=False, segment_color=(0, 255, 0)):
        """
        image: Input 3-channel image
        results: The output/detections from detect function
        all_at_once: If True, then ALL detected instances are returned in one image
                    If False, EACH detected instance is returned in an individual image
        segment_color: Color for segmenting the detected instances
        """
        r = results[0]
        N = self.num_instances_check(r)
        stitched = np.zeros((image.shape[0]*2, image.shape[1], image.shape[2]), dtype=np.uint8)

        orientations = []

        for i in range(r['masks'].shape[-1]):
            # Extract the mask for the current object
            mask = r['masks'][:, :, i]

            # Compute the bounding box
            bbox = utils.extract_bboxes(np.expand_dims(mask, -1))[0]
            y1, x1, y2, x2 = bbox

            # Compute the orientation of the bounding box
            orientation = self.compute_orientation(bbox)
            orientations.append(orientation)

            # Convert image to compatible format for OpenCV
            image_cv2 = np.uint8(image)
            
            # Draw the bounding box
            cv2.rectangle(image_cv2, (x1, y1), (x2, y2), segment_color, 2)

            # Draw the center point on the image
            center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
            cv2.circle(image_cv2, center, 3, segment_color, -1)  # Draw a circle at the center

            stitched[0:image.shape[0], :, :] = image_cv2
            stitched[image.shape[0]:, :, :] = image_cv2

            if not all_at_once:
                if self.dest_dir:
                    self.save_image(image_cv2, instance_idx=i+1, with_bbox=True)
                    self.save_image(image_cv2 * np.expand_dims(mask, -1), instance_idx=i+1, with_bbox=False)

        if all_at_once:
            if self.dest_dir:
                # Draw the center points on the stitched image
                for i in range(r['masks'].shape[-1]):
                    mask = r['masks'][:, :, i]
                    bbox = utils.extract_bboxes(np.expand_dims(mask, -1))[0]
                    y1, x1, y2, x2 = bbox
                    center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
                    cv2.circle(stitched, center, 3, segment_color, -1)  # Draw a circle at the center

                    # Compute the orientation of the bounding box
                    orientation = self.compute_orientation(bbox)

                    # Draw the bounding box
                    cv2.rectangle(stitched, (x1, y1), (x2, y2), segment_color, 2)

                    # Draw the orientation line
                    length = max(abs(x2 - x1), abs(y2 - y1)) // 2
                    x_center = (x1 + x2) // 2
                    y_center = (y1 + y2) // 2
                    x_end = int(x_center + length * np.cos(np.radians(orientation)))
                    y_end = int(y_center + length * np.sin(np.radians(orientation)))
                    cv2.line(stitched, (x_center, y_center), (x_end, y_end), segment_color, 2)

                print("Orientations of bounding boxes:", orientations)
                
        return stitched, center, orientations

class ImageCaptureAndDetectNode:
    def __init__(self):
        rospy.init_node('image_capture_and_detect_node', anonymous=True)
        self.publisher_capture_image = rospy.Publisher('    ', Image, queue_size=10)
        self.publisher_num_books = rospy.Publisher('/num_detected_books', Int32, queue_size=10)
        self.publisher_seg_image = rospy.Publisher('/seg_image', Image, queue_size=10)
        self.publisher_center = rospy.Publisher('/centre_box', Point, queue_size=10)
        self.publisher_orientation = rospy.Publisher('/orientation', Float32, queue_size=10)
        self.bridge = CvBridge()

        # Initialize RealSense pipeline
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Set the color stream parameters
        pipeline.start(config)

        self.pipeline = pipeline

        self.book_detector = BookDetector()

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

        try:
            # Publish the captured image
            ros_image = self.bridge.cv2_to_imgmsg(frame_data, "bgr8")
            self.publisher_capture_image.publish(ros_image)

            # Detect books in the captured image
            results, num_books = self.book_detector.detect(frame_data)
            num_books_msg = Int32()
            num_books_msg.data = num_books
            self.publisher_num_books.publish(num_books_msg)

            # Segment the image
            segmented_image, center, orientations = self.book_detector.segment(frame_data, results, all_at_once=True, show=False)

            # Publish segmented image
            segmented_image_msg = self.bridge.cv2_to_imgmsg(segmented_image, encoding="bgr8")
            self.publisher_seg_image.publish(segmented_image_msg)

            # Publish center point
            center_msg = Point()
            center_msg.x = center[0]
            center_msg.y = center[1]
            self.publisher_center.publish(center_msg)

            # Publish orientations
            orientation_msg = Float32()
            orientation_msg.data = orientations
            self.publisher_orientation.publish(orientation_msg)

        except CvBridgeError as e:
            rospy.logwarn('Failed to convert frame to ROS image: %s' % e)

def main():
    image_capture_and_detect_node = ImageCaptureAndDetectNode()
    rospy.spin()

if __name__ == '__main__':
    main()
