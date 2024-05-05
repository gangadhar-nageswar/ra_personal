import rospy
from sensor_msgs.msg import BoundingBox, Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import easyocr
import csv
import cv2

class BookShelfDetector:
    def __init__(self):
        rospy.init_node('book_shelf_detector', anonymous=True)
        self.ocr = easyocr.Reader(['en'])  # Initialize Easy OCR
        self.shelf_map = self.load_shelf_map()  # Load shelf map from CSV
        self.cv_bridge = CvBridge()
        self.bounding_boxes = {}  # Dictionary to store bounding boxes and corresponding text

        rospy.Subscriber('/capture_image', Image, self.image_callback)
        rospy.Subscriber('/bounding_box', BoundingBox, self.bounding_box_callback)
        self.shelf_pub = rospy.Publisher('/shelf_no', Int32MultiArray, queue_size=10)

    def load_shelf_map(self):
        shelf_map = {}
        with open('shelf_map.csv', 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                shelf_map[row[0]] = int(row[1])
        return shelf_map

    def image_callback(self, image_msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

        # Perform OCR on the entire image
        ocr_results = self.ocr.readtext(cv_image)

        # Merge OCR text within bounding boxes
        merged_text = self.merge_text_in_boxes(ocr_results)

        # Process OCR results, extract book names, and publish shelf numbers
        book_names = self.extract_book_names(merged_text)
        shelf_numbers = self.match_book_names_with_shelves(book_names)

        shelf_msg = Int32MultiArray(data=shelf_numbers)
        self.shelf_pub.pub  lish(shelf_msg)

    def merge_text_in_boxes(self, ocr_results):
        # Merge OCR text within bounding boxes
        merged_text = {}
        for result in ocr_results:
            box = result[0]
            text = result[1]
            if box in self.bounding_boxes:
                merged_text[box] += ' ' + text
            else:
                merged_text[box] = text
        return merged_text

    def extract_book_names(self, merged_text):
        book_names = []
        for text in merged_text.values():
            if self.is_book_name(text):
                book_names.append(text)
        return book_names

    def match_book_names_with_shelves(self, book_names):
        shelf_numbers = []
        for book_name in book_names:
            if book_name in self.shelf_map:
                shelf_numbers.append(self.shelf_map[book_name])
            else:
                shelf_numbers.append(-1)  # No shelf number found
        return shelf_numbers

    def bounding_box_callback(self, msg):
        # Store bounding box coordinates and corresponding text
        self.bounding_boxes[(msg.x1, msg.y1, msg.x2, msg.y2)] = msg.text

    def is_book_name(self, text):
        # Implement a logic to determine if a text is a book name
        # You might use heuristics or a more sophisticated approach
        if len(text) > 3 and any(c.isupper() for c in text):
            return True
        else:
            return False

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = BookShelfDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
