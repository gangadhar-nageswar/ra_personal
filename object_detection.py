# #!/usr/bin/env python

# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
from ultralytics import YOLO
# import numpy as np

# class BookDetectorROS:
#     def __init__(self):
#         rospy.init_node('book_detector_ros')

#         # Initialize CvBridge
#         self.bridge = CvBridge()

#         # Load YOLOv8 OBB model
#         self.model = YOLO('yolov8n-obb.pt')

        
#         self.bbox_pub = rospy.Publisher('Bounding_Box_8B', Image, queue_size=1)
#         self.center_pub = rospy.Publisher('Centre_Point_8B', Image, queue_size=1)
#         self.orientation_pub = rospy.Publisher('Orientation_matrix_8B', Image, queue_size=1)

        
#         self.image_sub = rospy.Subscriber('image_capture/Image', Image, self.image_callback)

#     def image_callback(self, msg):
#         try:
            
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         except CvBridgeError as e:
#             rospy.logerr("Error converting ROS Image message to OpenCV image: %s" % e)
#             return

#         # Predict using the YOLO model
#         results = self.model(frame)

#         # Filter results for 'book' class
#         book_class_id = results.names.index('book')  # Assuming 'book' is the class name for books
#         book_results = results.filter(class_id=book_class_id)

      
#         for obb in book_results.obb:
#             bbox = obb.xyxy.astype(np.int)  # Bounding box coordinates [x1, y1, x2, y2]
#             center = np.array([(bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2], dtype=np.int)  # Center point [x, y]
#             orientation_matrix = obb.orientation_matrix.astype(np.float32)  # Orientation matrix

           
#             bbox_msg = self.bridge.cv2_to_imgmsg(frame[bbox[1]:bbox[3], bbox[0]:bbox[2]], encoding="bgr8")
#             center_msg = self.bridge.cv2_to_imgmsg(np.array([center]), encoding="32FC1")
#             orientation_msg = self.bridge.cv2_to_imgmsg(orientation_matrix, encoding="32FC1")

#             self.bbox_pub.publish(bbox_msg)
#             self.center_pub.publish(center_msg)
#             self.orientation_pub.publish(orientation_msg)

# if __name__ == '__main__':
#     try:
#         BookDetectorROS()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
import cv2
from ultralytics import YOLO
import numpy as np

class BookDetector:
    def __init__(self, model_path='yolov8n-obb.pt'):
        self.model = YOLO(model_path)

    def detect_books(self, frame):
        results = self.model(frame)
        # Assuming 'book' is the class name for books
        print(self.model.names)
        book_class_id = self.model.names('book')
        
        book_results = [result for result in results if result['class'] == book_class_id]

        detected_books = []
        for obb in book_results:
            bbox = obb['box'].astype(np.int)  # Bounding box coordinates [x1, y1, x2, y2]
            center = np.array([(bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2], dtype=np.int)  # Center point [x, y]
            orientation_matrix = obb['orientation_matrix'].astype(np.float32)  # Orientation matrix

            detected_books.append({'bbox': bbox, 'center': center, 'orientation': orientation_matrix})

        return detected_books

class BookDetectionModule:
    def __init__(self, model_path='yolov8n-obb.pt'):
        self.book_detector = BookDetector(model_path)
        self.class_names = ["book"]

    def detect_and_visualize_books(self, image_path):
        frame = cv2.imread(image_path)

        detected_books = self.book_detector.detect_books(frame)

        for book in detected_books:
            bbox = book['bbox']
            center = book['center']
            orientation = book['orientation']

            # Draw bounding box
            cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)

            # Draw center point
            cv2.circle(frame, (center[0], center[1]), 5, (0, 0, 255), -1)

            # Print orientation matrix (just for demonstration)
            print("Orientation Matrix:", orientation)

        # Display the image with detected books
        cv2.imshow("Detected Books", frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__ == '__main__':
    # Example usage:
    book_module = BookDetectionModule()
    book_module.detect_and_visualize_books("test_image.jpeg")

