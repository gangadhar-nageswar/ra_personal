# #!/usr/bin/env python

# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# from std_msgs.msg import Int32, Float32
# from geometry_msgs.msg import Point
# import cv2
# import os
# import sys
# import numpy as np
# import skimage.io
# import argparse
# import tensorflow as tf
# from cv_bridge import CvBridge, CvBridgeError

# from mrcnn import utils
# import mrcnn.model as modellib
# import book  # Import BOOK config

# # Ignore depreciation warnings
# tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)

# class BookDetector:
#     def __init__(self, root_dir="./Mask_RCNN", model_path="models/mask_rcnn_book_0999.h5"):
#         # Root directory of the project
#         ROOT_DIR = os.path.abspath(root_dir)

#         # Import Mask RCNN
#         sys.path.append(ROOT_DIR)  # To find local version of the library

#         # Local path to trained weights file
#         BOOK_MODEL_PATH = "./models/mask_rcnn_book_0999.h5"
        
#         # Directory to save logs and trained model
#         MODEL_DIR = os.path.join(ROOT_DIR, "logs")
        
#         # Load config
#         config = book.BookConfig()

#         # Destination directory
#         self.dest_dir = "/home/student/16662_RobotAutonomy/src/devel_packages/team8B/Library_Book_Detection/result"

#         # Create model object in inference mode.
#         self.model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=config)

#         # Load weights trained on MS-COCO
#         self.model.load_weights(BOOK_MODEL_PATH, by_name=True)

#     # Check presence of detections
#     def num_instances_check(self, results):
#         # Number of instances
#         r = results
#         boxes = r['rois']
#         masks = r['masks']
#         class_ids = r['class_ids']
#         N = boxes.shape[0]
#         if not N:
#             print("\n*** No instances to display *** \n")
#             exit()
#         else:
#             assert boxes.shape[0] == masks.shape[-1] == class_ids.shape[0]
#         print('{} instances detected'.format(N))
#         return N

#     # Model forward pass
#     def detect(self, image):
#         # Run detection
#         results = self.model.detect([image], verbose=1)
#         N = self.num_instances_check(results[0])

#         return results, N

#     # Saves images at dest_dir
#     def save_image(self, image, original_file_path, instance_idx=0, with_bbox=False):
#         dot_idx = original_file_path.rfind('.')
#         slash_idx = original_file_path.rfind('/')
#         if slash_idx == -1:
#             tmp_name = original_file_path[:dot_idx]
#         else:
#             tmp_name = original_file_path[slash_idx+1:dot_idx]
#         extension = original_file_path[dot_idx:]

#         if instance_idx:
#             if with_bbox:
#                 new_file_path = os.path.join(self.dest_dir, tmp_name + '_bbox_' + str(instance_idx) + extension)
#             else:
#                 new_file_path = os.path.join(self.dest_dir, tmp_name + '_seg_' + str(instance_idx) + extension)
#         else:
#             if with_bbox:
#                 new_file_path = os.path.join(self.dest_dir, tmp_name + '_bbox_all' + extension)
#             else:
#                 new_file_path = os.path.join(self.dest_dir, tmp_name + '_seg_all' + extension)

#         image = Image.fromarray(image)
#         image.save(new_file_path)
#         print('Image {} saved as {}'.format(original_file_path, new_file_path))

#     # Compute the orientation of the bounding box
#     def compute_orientation(self, bbox):
#         """
#         Compute the orientation (angle) of the bounding box
#         :param bbox: Tuple containing (y1, x1, y2, x2) coordinates of the bounding box
#         :return: Orientation angle in degrees
#         """
#         y1, x1, y2, x2 = bbox
#         angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
#         return angle

#     # Segment detections, show, and save if required
    

#     def segment(self, image, results, all_at_once=True, show=False, segment_color=(0, 255, 0)):
#         """
#         image: Input 3-channel image
#         results: The output/detections from detect function
#         all_at_once: If True, then ALL detected instances are returned in one image
#                     If False, EACH detected instance is returned in an individual image
#         segment_color: Color for segmenting the detected instances
#         """
#         r = results[0]
#         N = self.num_instances_check(r)
#         stitched = np.zeros((image.shape[0]*2, image.shape[1], image.shape[2]), dtype=np.uint8)

#         orientations = []

#         for i in range(r['masks'].shape[-1]):
#             # Extract the mask for the current object
#             mask = r['masks'][:, :, i]

#             # Compute the bounding box
#             bbox = utils.extract_bboxes(np.expand_dims(mask, -1))[0]
#             y1, x1, y2, x2 = bbox

#             # Compute the orientation of the bounding box
#             orientation = self.compute_orientation(bbox)
#             orientations.append(orientation)

#             # Convert image to compatible format for OpenCV
#             image_cv2 = np.uint8(image)
            
#             # Draw the bounding box
#             cv2.rectangle(image_cv2, (x1, y1), (x2, y2), segment_color, 2)

#             # Draw the center point on the image
#             center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
#             cv2.circle(image_cv2, center, 3, segment_color, -1)  # Draw a circle at the center

#             stitched[0:image.shape[0], :, :] = image_cv2
#             stitched[image.shape[0]:, :, :] = image_cv2

#             if not all_at_once:
#                 if self.dest_dir:
#                     self.save_image(image_cv2, instance_idx=i+1, with_bbox=True)
#                     self.save_image(image_cv2 * np.expand_dims(mask, -1), instance_idx=i+1, with_bbox=False)

#         if all_at_once:
#             if self.dest_dir:
#                 # Draw the center points on the stitched image
#                 for i in range(r['masks'].shape[-1]):
#                     mask = r['masks'][:, :, i]
#                     bbox = utils.extract_bboxes(np.expand_dims(mask, -1))[0]
#                     y1, x1, y2, x2 = bbox
#                     center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
#                     cv2.circle(stitched, center, 3, segment_color, -1)  # Draw a circle at the center

#                     # Compute the orientation of the bounding box
#                     orientation = self.compute_orientation(bbox)

#                     # Draw the bounding box
#                     cv2.rectangle(stitched, (x1, y1), (x2, y2), segment_color, 2)

#                     # Draw the orientation line
#                     length = max(abs(x2 - x1), abs(y2 - y1)) // 2
#                     x_center = (x1 + x2) // 2
#                     y_center = (y1 + y2) // 2
#                     x_end = int(x_center + length * np.cos(np.radians(orientation)))
#                     y_end = int(y_center + length * np.sin(np.radians(orientation)))
#                     cv2.line(stitched, (x_center, y_center), (x_end, y_end), segment_color, 2)

#                 self.save_image(stitched, with_bbox=True)
#                 self.save_image(image_cv2 * r['masks'], with_bbox=False)
                
#                 print("Orientations of bounding boxes:", orientations)
                
#             if show:
#                 plt.imshow(stitched)
#                 plt.show()
#                 plt.axis('off')



# class BookDetectorROS:
#     def __init__(self):
#         rospy.init_node('book_detector_ros')
        
#         # ROS topics
#         self.image_sub = rospy.Subscriber('/capture_image', Image, self.image_callback)
#         self.bbox_pub = rospy.Publisher('/bounding_box', Point, queue_size=10)
#         self.center_pub = rospy.Publisher('/centre_box', Point, queue_size=10)
#         self.num_books_pub = rospy.Publisher('/num_detected_books', Int32, queue_size=10)
#         self.orientation_pub = rospy.Publisher('/orientation', Float32, queue_size=10)
#         self.seg_image_pub = rospy.Publisher('/seg_image', Image, queue_size=10)  # New topic for segmented image
        
#         # Initialize the book detector
#         self.book_detector = BookDetector()

#         # Initialize CvBridge
#         self.bridge = CvBridge()

#     def image_callback(self, img_msg):
#         # Convert ROS Image message to OpenCV image
#         print("ABC")
        
#         image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        
#         # Detect books in the image
#         results, N = self.book_detector.detect(image)
        
#         # Publish the number of detected books
#         num_books_msg = Int32()
#         num_books_msg.data = N
#         self.num_books_pub.publish(num_books_msg)
        
#         # Segment the image
#         segmented_image = self.book_detector.segment(image, results, all_at_once=True, show=False)
        
#         # Convert the segmented image to ROS Image message
#         segmented_image_msg = self.bridge.cv2_to_imgmsg(segmented_image, encoding="bgr8")
        
#         # Publish the segmented image
#         self.seg_image_pub.publish(segmented_image_msg)

# if __name__ == '__main__':
#     try:
#         detector = BookDetectorROS()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Point
import cv2
import os
import sys
import numpy as np
import skimage.io
import argparse
import tensorflow as tf
from cv_bridge import CvBridge, CvBridgeError

from mrcnn import utils
import mrcnn.model as modellib
import book  # Import BOOK config

# Ignore depreciation warnings
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
                
            if show:
                plt.imshow(stitched)
                plt.show()
                plt.axis('off')
                
        return stitched, center, orientations



class BookDetectorROS:
    def __init__(self):
        rospy.init_node('book_detector_ros')
        
        # ROS topics
        self.image_sub = rospy.Subscriber('/capture_image', Image, self.image_callback)
        
        self.bbox_pub = rospy.Publisher('/bounding_box', Point, queue_size=10)
        self.center_pub = rospy.Publisher('/centre_box', Point, queue_size=10)
        self.num_books_pub = rospy.Publisher('/num_detected_books', Int32, queue_size=10)
        self.orientation_pub = rospy.Publisher('/orientation', Float32, queue_size=10)
        self.seg_image_pub = rospy.Publisher('/seg_image', Image, queue_size=10)  # New topic for segmented image
        
        # Initialize the book detector
        self.book_detector = BookDetector()

        # Initialize CvBridge
        self.bridge = CvBridge()

    def image_callback(self, img_msg):
        # Convert ROS Image message to OpenCV image
        print("ABC")
        
        image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        
        # Detect books in the image
        results, N = self.book_detector.detect(image)
        
        # Publish the number of detected books
        num_books_msg = Int32()
        num_books_msg.data = N
        self.num_books_pub.publish(num_books_msg)
        
        # Segment the image
        segmented_image, center, orientations = self.book_detector.segment(image, results, all_at_once=True, show=False)
        
        # Convert the segmented image to ROS Image message
        segmented_image_msg = self.bridge.cv2_to_imgmsg(segmented_image, encoding="bgr8")
        
        # Publish the segmented image
        self.seg_image_pub.publish(segmented_image_msg)
        print(center)
        # for center in center:
        center_msg = Point()
        center_msg.x = center[0]
        center_msg.y = center[1]
        print(f"center msg: {center_msg}")
        self.center_pub.publish(center_msg)

        # Publish orientations
        # for orientation in orientations:
        orientation_msg = Float32()
        orientation_msg.data = orientations
        self.orientation_pub.publish(orientation_msg)

if __name__ == '__main__':
    try:
        detector = BookDetectorROS()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
