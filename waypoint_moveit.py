import rospy
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import PoseArray

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
    SAFE_PLACE = 6


class FrankaMoveIt:
    def __init__(self):
        # Create FrankaArm instance
        self.franka_arm_moveit_planner = MoveItPlanner()
        # self.franka_arm = FrankaArm()
        # rospy.init_node('book_pickup_node', anonymous=True)
        self.fsm_state = RobotState.RESET
        # # Subscribe to book pose
        book_start_pose_sub = rospy.Subscriber("book_start_pose", PoseArray, self.book_start_poses_callback)
        book_goal_pose_sub = rospy.Subscriber("book_goal_pose", PoseArray, self.book_goal_pose_callback)

        self.book_start_pose = None
        self.book_goal_pose = None

        self.counter = 0
    
    def book_start_poses_callback(self, msg):
        self.book_start_pose = msg

    def book_goal_pose_callback(self, msg):
        self.book_goal_pose = msg

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
        safe_distance = 0.001
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
        
        return desired_pose
        # print(f"desired pose: {desired_pose}")
        # self.franka_arm_moveit_planner.goto_pose(desired_pose)

    def place_book(self, desired_pose):
        # Move to the desired location to place the book
        # can be completed later, as of now just lift book
        
        self.move_arm(desired_pose)
        

    def main(self):

        # self.move_to_reset_position()
        # rospy.init_node('book_pickup_node')


        ################## For manual testing ######################
        if self.counter == 0:
            book_pose = Pose()
            '''Tra: [ 0.48777115 -0.03213804  0.12531104]
            Rot: [[ 0.99905414 -0.00309281 -0.04315146]
            [-0.00212477 -0.99973582  0.02246161]
            [-0.04320953 -0.02234868 -0.99881601]]
            Qtn: [-0.01120593  0.9997007  -0.00130479 -0.02159671]
            from franka_tool to world
            '''
            book_pose.position.x = 0.48777115    
            book_pose.position.y = -0.03213804
            book_pose.position.z = 0.12531104 
            book_pose.orientation.w = -0.01120593
            book_pose.orientation.x = 0.9997007 
            book_pose.orientation.y = -0.00130479
            book_pose.orientation.z = -0.02159671

        elif self.counter == 1:
            book_pose = Pose()
            '''Tra: [0.53961883 0.10497856 0.12884937]
            Rot: [[ 0.99875413 -0.03323408  0.03696562]
            [-0.0347699  -0.9985149   0.04171121]
            [ 0.03552448 -0.04294453 -0.99844565]]
            Qtn: [-0.02117528  0.99946419 -0.01701011  0.01813224]
            from franka_tool to world
            '''
            book_pose.position.x = 0.53961883  
            book_pose.position.y = 0.10497856
            book_pose.position.z = 0.12884937 
            book_pose.orientation.w = -0.02117528
            book_pose.orientation.x = 0.99946419
            book_pose.orientation.y = -0.01701011
            book_pose.orientation.z = 0.01813224

        

        #############################################################

        # self.move_to_safe_distance_above_book(book_pose)
        # self.grasp_book(0.2)


        ####################### Using subscriber ####################
        # book_pose = self.book_start_pose
        
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
            self.quat_alone = Pose()
            self.quat_alone = self.orient_book(current_pose)
            time.sleep(3)
            # self.fsm_state = RobotState.RESET
            
            self.fsm_state = RobotState.PLACE_BOOK


        elif self.fsm_state == RobotState.PLACE_BOOK:
            # Assuming desired location is fixed for now
            # desired_location = RigidTransform(translation=np.array([0.19859332, 0.56875431 ,0.44984054]), from_frame='franka_tool', to_frame='franka_tool')
            # desired_location = Pose()
            pose = self.franka_arm_moveit_planner.get_pose()
            if self.counter == 0:
                desired_pose = Pose()
                desired_pose.position.x = pose.translation[0]
                desired_pose.position.y = pose.translation[1] + 0.6
                desired_pose.position.z = pose.translation[2]
                desired_pose.orientation.w = pose.quaternion[0]
                desired_pose.orientation.x = pose.quaternion[1]
                desired_pose.orientation.y = pose.quaternion[2]
                desired_pose.orientation.z = pose.quaternion[3]
                

            elif self.counter == 1:
                desired_pose = Pose()
                desired_pose.position.x = pose.translation[0] 
                desired_pose.position.y = pose.translation[1] 
                desired_pose.position.z = pose.translation[2] - 0.3
                desired_pose.orientation.w = pose.quaternion[0]
                desired_pose.orientation.x = pose.quaternion[1]
                desired_pose.orientation.y = pose.quaternion[2]
                desired_pose.orientation.z = pose.quaternion[3]
                self.place_book(desired_pose)
                time.sleep(2)
                pose = self.franka_arm_moveit_planner.get_pose()

                desired_pose.position.x = pose.translation[0] 
                desired_pose.position.y = pose.translation[1] + 0.4
                desired_pose.position.z = pose.translation[2] 
                desired_pose.orientation.w = pose.quaternion[0]
                desired_pose.orientation.x = pose.quaternion[1]
                desired_pose.orientation.y = pose.quaternion[2]
                desired_pose.orientation.z = pose.quaternion[3]
            print(desired_pose)
            self.place_book(desired_pose)
            self.franka_arm_moveit_planner.open_gripper()
            self.fsm_state = RobotState.SAFE_PLACE
            time.sleep(3)
            
        elif self.fsm_state == RobotState.SAFE_PLACE:
            print("check enter")
            # time.sleep(2)
            pose = self.franka_arm_moveit_planner.get_pose()
            print(f"\ncurrent pose: \n{pose}")
            if self.counter == 0:
                desired_pos = Pose()
                desired_pos.position.x = pose.translation[0]
                desired_pos.position.y = pose.translation[1] - 0.2
                desired_pos.position.z = pose.translation[2]
                desired_pos.orientation.w = pose.quaternion[0]
                desired_pos.orientation.x = pose.quaternion[1]
                desired_pos.orientation.y = pose.quaternion[2]
                desired_pos.orientation.z = pose.quaternion[3]
                self.counter += 1


            elif self.counter == 1:
                desired_pos = Pose()
                desired_pos.position.x = pose.translation[0]
                desired_pos.position.y = pose.translation[1] - 0.5
                desired_pos.position.z = pose.translation[2]
                desired_pos.orientation.w = pose.quaternion[0]
                desired_pos.orientation.x = pose.quaternion[1]
                desired_pos.orientation.y = pose.quaternion[2]
                desired_pos.orientation.z = pose.quaternion[3]
                self.counter += 1


            
            print(f"\ndes pose: \n{desired_pos}")
            self.place_book(desired_pos)
            time.sleep(3)
            self.franka_arm_moveit_planner.reset_joints()
            self.fsm_state = RobotState.RESET
            self.book_pose = None


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
'''



'''########
point 1
pose: 
Tra: [ 0.53567952 -0.05637515  0.13231799]
 Rot: [[ 0.99182827 -0.11471992 -0.0556491 ]
 [-0.11621293 -0.99291565 -0.02436855]
 [-0.0524593   0.03063656 -0.99815297]]
 Qtn: [ 0.01378077  0.99785982 -0.05785704 -0.02708507]
 from franka_tool to world

 Goal 1:
 pose: 
Tra: [0.19859332 0.56875431 0.44984054]
 Rot: [[ 0.02535552 -0.99860546 -0.04609787]
 [-0.02438276 -0.04671632  0.99861054]
 [-0.99937147 -0.0241963  -0.02553376]]
 Qtn: [-0.48813559  0.52383337 -0.48822215 -0.49895129]
 from franka_tool to world

 Point 2:
 Tra: [0.53961883 0.10497856 0.12884937]
 Rot: [[ 0.99875413 -0.03323408  0.03696562]
 [-0.0347699  -0.9985149   0.04171121]
 [ 0.03552448 -0.04294453 -0.99844565]]
 Qtn: [-0.02117528  0.99946419 -0.01701011  0.01813224]
 from franka_tool to world

 Goal 2 (bottom):
 Tra: [0.28064021 0.56847877 0.09870181]
 Rot: [[ 0.02109886 -0.99924799 -0.03223446]
 [-0.01970358 -0.03265067  0.99927257]
 [-0.99957359 -0.02044838 -0.02037805]]
 Qtn: [-0.49195277  0.51820063 -0.49158159 -0.49778405]
'''