import rospy
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Int8

import numpy as np
from autolab_core import RigidTransform
from frankapy import FrankaArm

import sys
sys.path.append("/home/ros_ws")
from src.devel_packages.manipulation.src.moveit_class_modified import MoveItPlanner
from geometry_msgs.msg import Pose
import time 


# '''
# For MoveIt Class:
#     - pose argument should be of type geometry_msgs.msg.Pose
#     - get pose doesnt work with moveit -- change everything from scratch to frankapy 

# '''


class RobotState:
    RESET = 0
    MOVE_TO_BOOK = 1
    PICK_BOOK = 2
    LIFT_BOOK = 3
    ORIENT_BOOK = 4
    PLACE_BOOK = 5


class FrankaMoveIt:
    def __init__(self):
        # Create FrankaArm instance
        self.franka_arm_moveit_planner = MoveItPlanner()
        # self.franka_arm = FrankaArm()
        # rospy.init_node('book_pickup_node', anonymous=True)
        self.fsm_state = RobotState.RESET
        # # Subscribe to book pose
        self.book_poses_sub = rospy.Subscriber('/books_start_pose', PoseArray, self.book_pose_callback)
        self.book_category_sub = rospy.Subscriber('/books_category', Int8MultiArray, self.book_cat_callback)

        self.book_start_pose = None
        self.book_cat = None
        self.book_goal_pose = None


        # book manager code
        self.book_pose_array = []
        self.book_cat_array = []

        self.num_categories = 2
        self.shelf_occupancy1 = 0
        self.shelf_occupancy2 = 0

        self.goal_poses = [[pose11, pose12, pose13],
                           [pose21, pose22, pose23]]
    
    def book_pose_callback(self, msg):
        self.book_pose_array = msg.poses

        pickup_book_index = self.choose_pickup_book()
        self.book_start_pose = self.book_pose_array[pickup_book_index]
    
    def book_cat_callback(self, msg):
        self.book_cat_array = msg.data

        pickup_book_index = self.choose_pickup_book()
        cat = self.book_cat_array[pickup_book_index]

        self.book_goal_pose = self.goal_poses[cat][self.shelf_occupancy1 if cat == 0 else self.shelf_occupancy2]

        if cat == 0:
            self.shelf_occupancy1 += 1
        elif cat == 1:
            self.shelf_occupancy2 += 1
    
    def choose_pickup_book(self):
        # pick the book with the least x position value

        x_poses = [pose.position.x for pose in self.book_pose_array]
        min_x = min(x_poses)
        min_x_index = x_poses.index(min_x)

        return min_x_index

    def move_arm(self, pose):
        # convert pose goal to the panda_hand frame (the frame that moveit uses)
        pose_goal_moveit = self.franka_arm_moveit_planner.get_moveit_pose_given_frankapy_pose(pose)

        # plan a straight line motion to the goal
        plan = self.franka_arm_moveit_planner.get_plan_given_pose(pose_goal_moveit)

        # move arm to point in real-world
        self.franka_arm_moveit_planner.execute_plan(plan)
    
    def move_to_reset_position(self):
        # Move the robot to the reset position
        reset_pose = Pose()
        '''
        pose: 
        Tra: [0.52605034 0.03131209 0.50502002]
        Rot: [[ 0.99981857 -0.01039317 -0.01534812]
        [-0.01075839 -0.99964665 -0.02390779]
        [-0.01509422  0.02406857 -0.99959635]]
        Qtn: [ 0.0119955   0.99988269 -0.00528851 -0.00761148]
        '''
        reset_pose.position.x = 0.50861238
        reset_pose.position.y = 0.03076008
        reset_pose.position.z = 0.49726277
        reset_pose.orientation.w = 0.0119955
        reset_pose.orientation.x = 0.99988269
        reset_pose.orientation.y = 0.00528851
        reset_pose.orientation.z = -0.00761148

        self.move_arm(reset_pose)
        self.franka_arm_moveit_planner.open_gripper()
    
    def move_to_safe_distance_above_book(self, book_pose):
        # Calculate safe distance above the book
        safe_distance = 0.1
        gripper_width = 0.03

        # move to safe distance from book 
        desired_pose = book_pose
        desired_pose.position.z += safe_distance
        self.move_arm(desired_pose)

        # Adjust gripper width
        self.franka_arm_moveit_planner.goto_gripper(gripper_width)

    def grasp_book(self, gripper_width):
        # Close gripper to pick the book
        self.franka_arm_moveit_planner.close_gripper()
    
    def lift_book(self, pose , lift_height):
        # Lift the book vertically to a certain height
        desired_pose = Pose()
        desired_pose.position.x = pose.translation[0]
        desired_pose.position.y = pose.translation[1]
        desired_pose.position.z = pose.translation[2]
        desired_pose.orientation.w = pose.quaternion[0]
        desired_pose.orientation.x = pose.quaternion[1]
        desired_pose.orientation.y = pose.quaternion[2]
        desired_pose.orientation.z = pose.quaternion[3]

        print("check:", desired_pose)
        desired_pose.position.z += lift_height
        self.move_arm(desired_pose)

    def orient_book(self, current_pose):
        # Rotate the book 90 degrees
        # current_pose = self.franka_arm_moveit_planner.get_pose()
        rotation_matrix_z = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        rotation_matrix_x = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
        rotated_pose_x = RigidTransform(rotation=rotation_matrix_x, from_frame='franka_tool', to_frame='franka_tool')
        rotated_pose_z = RigidTransform(rotation=rotation_matrix_z, from_frame='franka_tool', to_frame='franka_tool')
        
        pose = (current_pose * rotated_pose_x) * rotated_pose_z

        desired_pose = Pose()
        desired_pose.position.x = pose.translation[0]
        desired_pose.position.y = pose.translation[1]
        desired_pose.position.z = pose.translation[2]
        desired_pose.orientation.w = pose.quaternion[0]
        desired_pose.orientation.x = pose.quaternion[1]
        desired_pose.orientation.y = pose.quaternion[2]
        desired_pose.orientation.z = pose.quaternion[3]

        self.move_arm(desired_pose)
        # print(f"desired pose: {desired_pose}")
        # self.franka_arm_moveit_planner.goto_pose(desired_pose)

    def place_book(self, desired_location):
        # Move to the desired location to place the book
        # can be completed later, as of now just lift book
        pass

    def main(self):

        # self.move_to_reset_position()
        # rospy.init_node('book_pickup_node')


        ################## For manual testing ######################
        # book_pose = Pose()
        # '''Tra: [0.51419988 0.0076894  0.11550828]
        # Rot: [[ 0.99813993  0.00954842 -0.06005263]
        # [ 0.00978518 -0.99993584  0.00364967]
        # [-0.06001393 -0.00423051 -0.99818854]]
        # Qtn: [-0.00197097  0.99953293  0.00483566 -0.03003066]
        # from franka_tool to world
        # '''
        # book_pose.position.x = 0.51419988    
        # book_pose.position.y = 0.0076894
        # book_pose.position.z = 0.11550828 
        # book_pose.orientation.w = -0.00197097
        # book_pose.orientation.x = 0.99953293
        # book_pose.orientation.y = 0.00483566
        # book_pose.orientation.z = -0.03003066
        #############################################################

        # self.move_to_safe_distance_above_book(book_pose)
        # self.grasp_book(0.2)


        ####################### Using subscriber ####################
        book_pose = self.book_start_pose
        desired_location = self.book_goal_pose
        
        if self.fsm_state == RobotState.RESET:
            self.move_to_reset_position()
            if book_pose is not None:
                self.fsm_state = RobotState.MOVE_TO_BOOK

        elif self.fsm_state == RobotState.MOVE_TO_BOOK:
            if book_pose is not None:
                self.move_to_safe_distance_above_book(book_pose)
                self.fsm_state = RobotState.PICK_BOOK

        elif self.fsm_state == RobotState.PICK_BOOK:
            # Assuming gripper width and lift height are fixed
            gripper_width = 0.03
            lift_height = 0.3
            self.grasp_book(gripper_width)
            current_pose = self.franka_arm_moveit_planner.get_pose()
            if current_pose is not None:
                self.lift_book(current_pose, lift_height)
                self.fsm_state = RobotState.ORIENT_BOOK
                time.sleep(3)

        elif self.fsm_state == RobotState.ORIENT_BOOK:
            current_pose = self.franka_arm_moveit_planner.get_pose()
            self.orient_book(current_pose)
            self.fsm_state = RobotState.PLACE_BOOK

        # elif self.fsm_state == RobotState.PLACE_BOOK:
        #     # Assuming desired location is fixed for now
        #     desired_location = RigidTransform(translation=np.array([0.5, -0.5, 0.5]), from_frame='world', to_frame='franka_tool')
        #     self.place_book(desired_location)
        #     self.fsm_state = RobotState.RESET
        #     self.book_pose = None


if __name__ == "__main__":

    fa = FrankaMoveIt()
    rate = rospy.Rate(10)

    # define collision boxes
    # mid layer
    box_pose = PoseStamped()
    box_pose.header.frame_id = "panda_link0"
    box_pose.pose.position.x = 0.388
    box_pose.pose.position.y = 0.688
    box_pose.pose.position.z = 0.290
    box_pose.pose.orientation.x = 0.0
    box_pose.pose.orientation.y = 0.0
    box_pose.pose.orientation.z = 0.0
    box_pose.pose.orientation.w = 1.0
    fa.franka_arm_moveit_planner.add_box("shelf_mid", box_pose, [0.538, 0.230, 0.015])

    # top layer
    box_pose = PoseStamped()
    box_pose.header.frame_id = "panda_link0"
    box_pose.pose.position.x = 0.388
    box_pose.pose.position.y = 0.688
    box_pose.pose.position.z = 0.580
    box_pose.pose.orientation.x = 0.0
    box_pose.pose.orientation.y = 0.0
    box_pose.pose.orientation.z = 0.0
    box_pose.pose.orientation.w = 1.0
    fa.franka_arm_moveit_planner.add_box("shelf_top", box_pose, [0.538, 0.230, 0.015])

    # outmost layer
    box_pose = PoseStamped()
    box_pose.header.frame_id = "panda_link0"
    box_pose.pose.position.x = 0.660
    box_pose.pose.position.y = 0.688
    box_pose.pose.position.z = 0.290
    box_pose.pose.orientation.x = 0.0
    box_pose.pose.orientation.y = 0.0
    box_pose.pose.orientation.z = 0.0
    box_pose.pose.orientation.w = 1.0
    fa.franka_arm_moveit_planner.add_box("shelf_outer", box_pose, [0.015, 0.290, 0.600])

    # innermost layer
    box_pose = PoseStamped()
    box_pose.header.frame_id = "panda_link0"
    box_pose.pose.position.x = 0.117
    box_pose.pose.position.y = 0.688
    box_pose.pose.position.z = 0.290
    box_pose.pose.orientation.x = 0.0
    box_pose.pose.orientation.y = 0.0
    box_pose.pose.orientation.z = 0.0
    box_pose.pose.orientation.w = 1.0
    fa.franka_arm_moveit_planner.add_box("shelf_inner", box_pose, [0.015, 0.290, 0.600])



    while not rospy.is_shutdown():
        fa.main()
        rate.sleep()
    
    rospy.spin()



'''
Reset position:
pose: 
Tra: [0.50861238 0.03076008 0.49726277]
 Rot: [[ 0.98314477  0.06102189  0.17229045]
 [ 0.05327279 -0.99735498  0.04925279]
 [ 0.17484024 -0.03924423 -0.98381409]]
 Qtn: [-0.02222361  0.99552924  0.02870199  0.0871724 ]
 from franka_tool to world

joints: 
[ 0.18084422 -0.27436572 -0.11699352 -1.97996796  0.00659837  1.883443
  0.79184353]


========================================================================
Define collision boxes:

1. Middle layer:
centroid pose:
x = 0.388
y = 0.688
z = 0.290


dim_x = 0.538
dim_y = 0.230
dim_z = 0.015


2. Top layer:
centroid pose:
x = 0.388
y = 0.688
z = 0.580


dim_x = 0.538
dim_y = 0.230
dim_z = 0.015

3. Side layer outmost:
centroid pose:
x = 0.660
y = 0.688
z = 0.290


dim_x = 0.015
dim_y = 0.290
dim_z = 0.600

4. Side layer innermost:
centroid pose:
x = 0.117
y = 0.230
z = 0.290


dim_x = 0.015
dim_y = 0.290
dim_z = 0.600

'''