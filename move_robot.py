import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

import numpy as np
from autolab_core import RigidTransform
from frankapy import FrankaArm

import sys
sys.path.append("/home/ros_ws")
from src.devel_packages.manipulation.src.moveit_class import MoveItPlanner
from geometry_msgs.msg import Pose
import time 


# '''
# For MoveIt Class:
#     - pose argument should be of type geometry_msgs.msg.Pose

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
        # self.franka_arm_moveit_planner = MoveItPlanner()
        self.franka_arm = FrankaArm()
        # rospy.init_node('book_pickup_node', anonymous=True)
        self.fsm_state = RobotState.RESET
        # # Subscribe to book pose
        # book_start_pose_sub = rospy.Subscriber("book_start_pose_sub", PoseArray, self.book_start_poses_callback)
        # book_goal_pose_sub = rospy.Subscriber("book_goal_pose_sub", PoseArray, self.book_goal_pose_callback)
    
    def book_start_poses_callback(self, msg):
        # for now just pick the first book
        if len(msg.poses) > 0:
            self.books_queue = msg.poses
            self.book_pose = self.books_queue[0]

    def book_goal_pose_callback(self, msg):
        pass

    def move_arm(self, pose):
        # convert pose goal to the panda_hand frame (the frame that moveit uses)
        # pose_goal_moveit = self.franka_arm_moveit_planner.get_moveit_pose_given_frankapy_pose(pose)
        # plan a straight line motion to the goal
        # plan = self.franka_arm_moveit_planner.get_plan_given_pose(pose_goal_moveit)
        # move arm to point in real-world
        # self.franka_arm_moveit_planner.execute_plan(plan)
        self.franka_arm.goto_pose(pose)
    
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
        self.franka_arm.open_gripper()
    
    def move_to_safe_distance_above_book(self, book_pose):
        # Calculate safe distance above the book
        safe_distance = 0.1
        gripper_width = 0.03

        # move to safe distance from book 
        desired_pose = book_pose
        desired_pose.position.z += safe_distance
        self.move_arm(desired_pose)

        # Adjust gripper width
        self.franka_arm.goto_gripper(gripper_width)

    def grasp_book(self, gripper_width):
        # Close gripper to pick the book
        self.franka_arm.close_gripper()
    
    def lift_book(self, pose , lift_height):
        # Lift the book vertically to a certain height
        desired_pose = pose
        print("check des:", desired_pose)
        desired_pose.position.z += lift_height
        self.move_arm(desired_pose)

    def orient_book(self):
        # Rotate the book 90 degrees
        # current_pose = self.franka_arm.get_pose()
        # rotation_matrix = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        # rotated_pose = RigidTransform(rotation=rotation_matrix, from_frame='franka_tool', to_frame='franka_tool')
        # desired_pose = current_pose * rotated_pose
        # self.franka_arm.goto_pose(desired_pose)
        pass

    def place_book(self, desired_location):
        # Move to the desired location to place the book
        # can be completed later, as of now just lift book
        pass

    def main(self):

        # self.move_to_reset_position()
        # rospy.init_node('book_pickup_node')
        book_pose = Pose()
        '''Tra: [0.51419988 0.0076894  0.11550828]
        Rot: [[ 0.99813993  0.00954842 -0.06005263]
        [ 0.00978518 -0.99993584  0.00364967]
        [-0.06001393 -0.00423051 -0.99818854]]
        Qtn: [-0.00197097  0.99953293  0.00483566 -0.03003066]
        from franka_tool to world
        '''
        book_pose.position.x = 0.51419988    
        book_pose.position.y = 0.0076894
        book_pose.position.z = 0.11550828 
        book_pose.orientation.w = -0.00197097
        book_pose.orientation.x = 0.99953293
        book_pose.orientation.y = 0.00483566
        book_pose.orientation.z = -0.03003066

        # self.move_to_safe_distance_above_book(book_pose)
        # self.grasp_book(0.2)
        
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
            current_pose = self.franka_arm.get_pose()
            print("check:", current_pose)
            if current_pose is not None:
                self.lift_book(current_pose, lift_height)
                self.fsm_state = RobotState.ORIENT_BOOK


        # elif self.fsm_state == RobotState.ORIENT_BOOK:
        #     self.orient_book()
        #     self.fsm_state = RobotState.PLACE_BOOK

        # elif self.fsm_state == RobotState.PLACE_BOOK:
        #     # Assuming desired location is fixed for now
        #     desired_location = RigidTransform(translation=np.array([0.5, -0.5, 0.5]), from_frame='world', to_frame='franka_tool')
        #     self.place_book(desired_location)
        #     self.fsm_state = RobotState.RESET
        #     self.book_pose = None


if __name__ == "__main__":

    fa = FrankaMoveIt()
    rate = rospy.Rate(10)

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


'''