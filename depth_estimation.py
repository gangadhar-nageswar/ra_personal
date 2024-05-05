
import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import ros_numpy

class DepthEstimation:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, msg_Image, self.imageDepthCallback)

    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            pix0, pix1 = (data.width//2, data.height//2)
            print(f'Center point: ({pix0}, {pix1})')
        except CvBridgeError as e:
            print(e)
            return

        # Convert the center point to 3D using the depth information
        depth_image_topic = '/camera/depth/image_raw'  # Assuming the depth image topic
        depth_image_msg = rospy.wait_for_message(depth_image_topic, msg_Image)
        depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, depth_image_msg.encoding)
        depth_at_center = depth_image[pix1, pix0]  # Depth at the center point

        print(f'Depth at center: {depth_at_center}(mm)')

if __name__ == '__main__':
    rospy.init_node("depth_estimation_8B")
    topic = 'Centre_point_8B'  # Change the topic to Centre_point_8B
    listener = DepthEstimation(topic)
    rospy.spin()


