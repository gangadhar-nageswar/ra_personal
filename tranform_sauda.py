import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, Vector3Stamped, QuaternionStamped, PoseStamped
from tf2_msgs.msg import TFMessage

class MarkerTransformer:
    def __init__(self):
        rospy.init_node('marker_transformer', anonymous=True)

        # Initialize TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.marker_timer = rospy.Timer(rospy.Duration(0.1), self.get_marker_transform)
        self.latest_marker_pose = None

        

    def get_marker_transform(self,event):
        try:
            # Lookup the transform from marker1_frame to panda_link0
            transform = self.tf_buffer.lookup_transform('panda_link0', 'camera_color_optical_frame', rospy.Time(0), rospy.Duration(0.5))

            self.latest_marker_pose = PoseStamped()
            # transform = TransformStamped()
            # rospy.loginfo("Transform from marker1_frame to panda_link0:\n{}".format(transform))
            self.latest_marker_pose.header.stamp = rospy.Time.now()
            self.latest_marker_pose.header.frame_id = 'panda_link0'
            self.latest_marker_pose.pose.position = transform.transform.translation
            self.latest_marker_pose.pose.orientation = transform.transform.rotation

            print(self.latest_marker_pose.pose.position)

            # transform = self.tf_buffer.lookup_transform('panda_link0', 'marker_2_frame', rospy.Time(0), rospy.Duration(0.5))
            # rospy.loginfo("Transform from marker1_frame to panda_link0:\n{}".format(transform))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.latest_marker_pose = None
            rospy.logerr("Failed to lookup transform: {}".format(e))

if __name__ == '__main__':
    try:
        marker_transformer = MarkerTransformer()
        rospy.spin()
      #   marker_transformer.get_marker_transform()
    except rospy.ROSInterruptException:
        pass