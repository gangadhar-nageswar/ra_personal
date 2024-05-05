import cv2
import numpy as np

def find_rectangles(image_path):
    # Load image
    image = cv2.imread(image_path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Detect edges
    edges = cv2.Canny(blur, 50, 150)

    # Find contours
    contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    rectangles = []

    for contour in contours:
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)

        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(approx)
            rectangles.append((x, y, w, h))

    return rectangles

def find_centers(rectangles):
    centers = []
    for rectangle in rectangles:
        x, y, w, h = rectangle
        center_x = x + w // 2
        center_y = y + h // 2
        centers.append((center_x, center_y))
    return centers

def find_orientations(image, rectangles):
    orientations = []
    for rectangle in rectangles:
        x, y, w, h = rectangle
        rect = cv2.minAreaRect(np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h]]))
        angle = rect[-1]
        orientations.append(angle)
    return orientations

def draw_rectangles(image, rectangles):
    for rectangle in rectangles:
        x, y, w, h = rectangle
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    return image

def main():
    image_path = 'test_image.jpeg'
    rectangles = find_rectangles(image_path)
    image = cv2.imread(image_path)
    image_with_rectangles = draw_rectangles(image.copy(), rectangles)
    centers = find_centers(rectangles)
    orientations = find_orientations(image, rectangles)

    for center, angle in zip(centers, orientations):
        print("Center:", center)
        print("Orientation:", angle)

    cv2.imshow("Image with rectangles", image_with_rectangles)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
#!/usr/bin/env python

# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# from geometry_msgs.msg import Point, Quaternion
# from std_msgs.msg import Float64

# class RectangleDetectorNode:
#     def __init__(self):
#         rospy.init_node('rectangle_detector_node', anonymous=True)
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber('/image_capture', Image, self.image_callback)
#         self.center_pub = rospy.Publisher('/rectangle_center', Point, queue_size=10)
#         self.orientation_pub = rospy.Publisher('/rectangle_orientation', Float64, queue_size=10)
#         self.bounding_box_pub = rospy.Publisher('/rectangle_bounding_box', Point, queue_size=10)

#     def image_callback(self, msg):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         except Exception as e:
#             rospy.logerr("Error converting image: %s", str(e))
#             return

#         rectangles = self.find_rectangles(cv_image)
#         centers = self.find_centers(rectangles)
#         orientations = self.find_orientations(cv_image, rectangles)

#         for center, orientation, rectangle in zip(centers, orientations, rectangles):
#             center_msg = Point()
#             center_msg.x, center_msg.y = center
#             self.center_pub.publish(center_msg)

#             orientation_msg = Float64()
#             orientation_msg.data = orientation
#             self.orientation_pub.publish(orientation_msg)

#             bounding_box_msg = Point()
#             bounding_box_msg.x, bounding_box_msg.y, bounding_box_msg.z, _ = rectangle
#             self.bounding_box_pub.publish(bounding_box_msg)

#     def find_rectangles(self, image):
#         # Your rectangle detection code here
#         pass

#     def find_centers(self, rectangles):
#         # Your center calculation code here
#         pass

#     def find_orientations(self, image, rectangles):
#         # Your orientation estimation code here
#         pass

# def main():
#     rectangle_detector_node = RectangleDetectorNode()
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         rospy.loginfo("Shutting down")

# if __name__ == '__main__':
#     main()
