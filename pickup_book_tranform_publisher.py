import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from tf2_geometry_msgs import do_transform_point

class PointTransformer:
    def __init__(self):
        rospy.init_node('point_transformer')

        # Initialize TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscriber for the point in camera_color_optical_frame
        # self.point_sub = rospy.Subscriber('/point_in_camera_frame', PointStamped, self.point_callback)
        self.point_sub = rospy.Subscriber('/centre_box', Point, self.point_callback)

        # Publisher for the transformed pose in panda_link0 frame
        self.pose_pub = rospy.Publisher('/transformed_point_in_panda_link0', PoseStamped, queue_size=10)

    def point_callback(self, point_msg):
        try:
            # Get the transform from camera_color_optical_frame to panda_link0
            # transform = self.tf_buffer.lookup_transform('panda_link0', point_msg.header.frame_id, rospy.Time(0))
            transform = self.tf_buffer.lookup_transform('panda_link0', "camera_color_optical_frame", rospy.Time(0))

            # Transform the point to panda_link0 frame
            transformed_point = do_transform_point(point_msg, transform)

            # Create PoseStamped message for the transformed point
            transformed_pose = PoseStamped()
            transformed_pose.header.frame_id = 'panda_link0'
            transformed_pose.header.stamp = rospy.Time.now()
            transformed_pose.pose.position = transformed_point.point
            transformed_pose.pose.orientation.w = 1.0  # Assuming orientation is not provided

            # Publish the transformed pose
            self.pose_pub.publish(transformed_pose)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Transform lookup failed: %s", e)

if __name__ == '__main__':
    try:
        PointTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
