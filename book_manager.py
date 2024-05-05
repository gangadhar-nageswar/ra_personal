import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Int8


'''
FSM States:
    -1: Reset
    0: move to home position
    1: pick up book
    2: move to book
    3: grasp book
    4: move to drop off location
'''



class BookManager:
    def __init__(self):
        self.book_pose_array = []
        self.book_cat = []
        
        self.pickup_books_queue = []
        self.drop_books_queue = []

        self.fsm_state = None

        self.book_poses_sub = rospy.Subscriber('/books_pose', PoseArray, self.book_pose_callback)
        self.book_category_sub = rospy.Subscriber('/books_category', Int8MultiArray, self.book_cat_callback)
        self.fsm_sub = rospy.Subscriber('/fsm', Int8, self.fsm_callback)

        self.pickup_book_pub = rospy.Publisher('/pickup_book', Pose, queue_size=10)
        self.drop_book_pub = rospy.Publisher('/drop_book', Pose, queue_size=10)

        self.num_categories = 3
        self.shelf_occupancy = [0 for i in range(self.num_categories)]
    
    def book_pose_callback(self, msg):
        self.book_pose_array = msg.poses
    
    def book_cat_callback(self, msg):
        self.book_cat = msg.data
    
    def fsm_callback(self, msg):
        self.fsm_state = msg.data

        if self.fsm_state == 0:
            self.pickup_books_queue = []
            self.drop_books_queue = []
        
        if self.fsm_state == 1:
            for i in range(len(self.book_cat)):
                if self.book_cat[i] == 1:
                    self.pickup_books_queue.append(self.book_pose_array[i])

    def add_book(self, book_pose):
        self.book_pose_array.poses.append(book_pose)

    def remove_book(self, book_pose):
        self.book_pose_array.poses.remove(book_pose)

    def update_book_status(self, book_status):
        self.book_status.data = book_status

    def get_book_pose(self):
        return self.book_pose

    def get_book_pose_array(self):
        return self.book_pose_array

    def get_book_status(self):
        return self.book_status

    def get_book_status_at_index(self, index):
        return self.book_status.data[index]

    def get_book_pose_at_index(self, index):
        return self.book_pose_array.poses[index]

    def get_num_books(self):
        return len(self.book_pose_array.poses)

    def clear_books(self):
        self.book_pose_array.poses = []
        self.book_status.data = []
