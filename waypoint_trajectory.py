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
from scipy.spatial.transform import Rotation as R

import ipdb
st = ipdb.set_trace

import copy


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
        self.lift_height = 0.3
        self.safe_drop_dist = 0.2
    
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
        return reset_pose

    def generate_tajectory(self, start_pose, goal_pose):

        # st()

        # step 0: Start pose 
        waypoint = []
        waypoint.append(start_pose)

        # step 1: lift book from the start pose
        lift_pose = copy.deepcopy(start_pose)
        lift_pose.position.z += self.lift_height
        waypoint.append(lift_pose)
        
        # step 2: orient the book 
        reorient_pose = self.orient_book(lift_pose)
        waypoint.append(reorient_pose)

        # step 3: move to the goal Z point
        z_move = copy.deepcopy(reorient_pose)
        z_move.position.z = goal_pose.position.z
        waypoint.append(z_move)

        # step 4: move to the goal X point
        x_move = copy.deepcopy(z_move)
        x_move.position.x = goal_pose.position.x 
        waypoint.append(x_move)

        # step 5: move to the goal Y point
        y_move = copy.deepcopy(x_move)
        y_move.position.y = goal_pose.position.y
        waypoint.append(y_move)

        # step 6: move to safe distance from the book placed 
        y_safe = copy.deepcopy(y_move)
        y_safe.position.y = y_move.position.y - self.safe_drop_dist
        waypoint.append(y_safe)

        # step 7: reset 
        reset_pose = self.move_to_reset_position
        waypoint.append(reset_pose)

        return waypoint
        

    def grasp_book(self, gripper_width):
        # Close gripper to pick the book
        self.franka_arm_moveit_planner.close_gripper()
    

    def orient_book(self, current_pose):
        # st()
        # Rotate the book 90 degrees
        # current_pose = self.franka_arm_moveit_planner.get_pose()
        rotation_matrix_z = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        rotation_matrix_x = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]])

        # Convert the current_pose from Pose() to a transformation matrix
        current_translation = np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z, 1])
        current_rotation = R.from_quat([current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w])
        current_rotation_matrix = current_rotation.as_matrix()
        current_pose_matrix = np.eye(4)
        current_pose_matrix[:3, :3] = current_rotation_matrix
        current_pose_matrix[:3, 3] = current_translation[:3]

        # Apply rotations
        rotated_pose_matrix = np.dot(rotation_matrix_z, current_pose_matrix)
        rotated_pose_matrix = np.dot(rotation_matrix_x, rotated_pose_matrix)

        # Extract translation and quaternion from the rotated_pose_matrix
        rotated_translation = rotated_pose_matrix[:3, 3]
        rotated_quaternion = R.from_matrix(rotated_pose_matrix[:3, :3]).as_quat()


        # Create a new Pose object for the rotated pose
        desired_pose = Pose()
        desired_pose.position.x = current_translation[0]
        desired_pose.position.y = current_translation[1]
        desired_pose.position.z = current_translation[2]
        desired_pose.orientation.x = rotated_quaternion[0]
        desired_pose.orientation.y = rotated_quaternion[1]
        desired_pose.orientation.z = rotated_quaternion[2]
        desired_pose.orientation.w = rotated_quaternion[3]

        return desired_pose

    def main(self):


        ################## For manual testing ######################
        if self.counter == 0:
            start_pose = Pose()
            book_pose = Pose()
            '''
            Green
            pose: 
            Tra: [0.48609779 0.08574463 0.11343034]
            Rot: [[ 0.98984891  0.06763175 -0.12492452]
            [ 0.06990401 -0.99744735  0.01389102]
            [-0.12366615 -0.02248274 -0.992069  ]]
            Qtn: [-0.00911699  0.99741732  0.03447297 -0.06230859]
            from franka_tool to world

            pose: 
            Tra: [0.2542272  0.54131726 0.3930374 ]
            Rot: [[ 8.66833608e-03  9.96778072e-01 -7.96191209e-02]
            [-1.13968918e-01  8.00873401e-02  9.90250842e-01]
            [ 9.93436809e-01  4.90277943e-04  1.14298143e-01]]
            Qtn: [ 0.54841905 -0.45118808 -0.48915876 -0.50634045]
            from franka_tool to world

            '''
            start_pose.position.x = 0.48609779  
            start_pose.position.y = 0.08574463
            start_pose.position.z = 0.11343034 
            start_pose.orientation.w = -0.01120593
            start_pose.orientation.x = 0.9997007 
            start_pose.orientation.y = -0.00130479
            start_pose.orientation.z = -0.02159671

            book_pose.position.x = 0.2542272
            book_pose.position.y = 0.54131726
            book_pose.position.z = 0.3930374
            book_pose.orientation.x = 0.0
            book_pose.orientation.y = 0.0
            book_pose.orientation.z = 0.0
            book_pose.orientation.w = 1.0

        elif self.counter == 1:
            start_pose = Pose()
            book_pose = Pose()
            '''
            Blue
            pose: 
            Tra: [ 0.6568081  -0.14643776  0.1201927 ]
            Rot: [[ 0.9911468   0.08715405 -0.10006563]
            [ 0.07762959 -0.99239507 -0.09542848]
            [-0.10762162  0.08681558 -0.990394  ]]
            Qtn: [ 0.04571018  0.99673666  0.04133079 -0.05209181]
            from franka_tool to world

            pose: 
            Tra: [0.40602605 0.55371199 0.3968626 ]
            Rot: [[ 0.02818015  0.99137705 -0.12790025]
            [-0.06630993  0.12952204  0.98935667]
            [ 0.9973914  -0.01939951  0.06938948]]
            Qtn: [ 0.55387085 -0.45532103 -0.5079215  -0.47740686]
            from franka_tool to world
            '''
            start_pose.position.x = 0.6568081  
            start_pose.position.y = -0.14643776
            start_pose.position.z = 0.1201927 
            start_pose.orientation.w = -0.01120593
            start_pose.orientation.x = 0.9997007 
            start_pose.orientation.y = -0.00130479
            start_pose.orientation.z = -0.02159671

            book_pose.position.x = 0.40602605
            book_pose.position.y = 0.55371199
            book_pose.position.z = 0.3968626
            book_pose.orientation.x = 0.0
            book_pose.orientation.y = 0.0
            book_pose.orientation.z = 0.0
            book_pose.orientation.w = 1.0
        
        elif self.counter == 2:
            start_pose = Pose()
            book_pose = Pose()
            '''
            orange
            pose: 
            Tra: [ 0.66207908 -0.00166629  0.11501748]
            Rot: [[ 0.99692476  0.04855369 -0.06135462]
            [ 0.04889357 -0.99878615  0.00404968]
            [-0.06108352 -0.00703707 -0.99810782]]
            Qtn: [-0.00277383  0.99922704  0.02438066 -0.03063321]
            from franka_tool to world

            pose: 
            Tra: [0.25881709 0.55905988 0.11613355]
            Rot: [[ 0.0720826   0.98931157 -0.12667977]
            [-0.08482866  0.13262977  0.98752872]
            [ 0.9937751  -0.06043756  0.09348407]]
            Qtn: [ 0.56969212 -0.45988273 -0.49169316 -0.47136874]
            from franka_tool to world
            '''

            start_pose.position.x = 0.47768965  
            start_pose.position.y = -0.08509741
            start_pose.position.z = 0.11772312 
            start_pose.orientation.w = -0.01120593
            start_pose.orientation.x = 0.9997007 
            start_pose.orientation.y = -0.00130479
            start_pose.orientation.z = -0.02159671

            book_pose.position.x = 0.25881709
            book_pose.position.y = 0.55905988
            book_pose.position.z = 0.11613355
            book_pose.orientation.x = 0.0
            book_pose.orientation.y = 0.0
            book_pose.orientation.z = 0.0
            book_pose.orientation.w = 1.0
        
        elif self.counter == 3:
            start_pose = Pose()
            book_pose = Pose()
            '''
            yellow
            pose: 
            Tra: [ 0.47768965 -0.08509741  0.11772312]
            Rot: [[ 0.99244266 -0.01087249 -0.12214908]
            [-0.00938557 -0.99986514  0.01274195]
            [-0.12227115 -0.01149922 -0.99242997]]
            Qtn: [-0.00607189  0.9980904  -0.00507421 -0.06122197]
            from franka_tool to world

            pose: 
            Tra: [0.40016891 0.56485408 0.11201383]
            Rot: [[ 0.05408492  0.99689113 -0.0571289 ]
            [-0.13564292  0.06401679  0.98868722]
            [ 0.98927072 -0.04572394  0.13868624]]
            Qtn: [ 0.56053277 -0.46135178 -0.46669868 -0.505115  ]
            from franka_tool to world
            '''

            start_pose.position.x = 0.66207908  
            start_pose.position.y = -0.00166629
            start_pose.position.z = 0.11501748 
            start_pose.orientation.w = -0.01120593
            start_pose.orientation.x = 0.9997007 
            start_pose.orientation.y = -0.00130479
            start_pose.orientation.z = -0.02159671

            book_pose.position.x = 0.40016891
            book_pose.position.y = 0.56485408
            book_pose.position.z = 0.11201383
            book_pose.orientation.x = 0.0
            book_pose.orientation.y = 0.0
            book_pose.orientation.z = 0.0
            book_pose.orientation.w = 1.0

        

        #############################################################


        # st()
        if self.fsm_state == RobotState.RESET:
            self.move_to_reset_position()
            pose = Pose()
            if book_pose is not None:
                self.fsm_state = RobotState.MOVE_TO_BOOK
                self.points = self.generate_tajectory(start_pose= start_pose, goal_pose=book_pose)
                print("points assigned :", self.points)

        elif self.fsm_state == RobotState.MOVE_TO_BOOK:
            # st()
            if book_pose is not None:
                pose = self.points[0]
                self.move_arm(pose)
                time.sleep(3)
                self.fsm_state = RobotState.PICK_BOOK

        elif self.fsm_state == RobotState.PICK_BOOK:
            # st()
            # Assuming gripper width and lift height are fixed
            gripper_width = 0.03
            self.grasp_book(gripper_width)
            pose = self.points[1]
            self.move_arm(pose)
            time.sleep(3)
            self.fsm_state = RobotState.ORIENT_BOOK

        elif self.fsm_state == RobotState.ORIENT_BOOK:
            # st()
            pose = self.points[2]
            
            self.move_arm(pose)
            time.sleep(3)       
            self.fsm_state = RobotState.PLACE_BOOK


        elif self.fsm_state == RobotState.PLACE_BOOK:
            # st()
            # Move to goal Z
            print("moving to Z")
            pose = self.points[3]
            self.move_arm(pose)
            time.sleep(3)

            # Move to goal X
            print("moving to X")
            pose = self.points[4]
            self.move_arm(pose)
            time.sleep(3)

            # Move to goal Y
            print("moving to Y")
            pose = self.points[5]
            self.move_arm(pose)
            time.sleep(3)

            self.fsm_state = RobotState.SAFE_PLACE
            
        elif self.fsm_state == RobotState.SAFE_PLACE:
            print("check enter")
            # time.sleep(2)
            # Open Gripper to drop book 
            self.franka_arm_moveit_planner.open_gripper()
            time.sleep(2)

            # Move to safe distance from drop point 
            pose = self.points[6]
            self.move_arm(pose)
            time.sleep(3)

            self.franka_arm_moveit_planner.reset_joints()
            self.fsm_state = RobotState.RESET
            self.book_pose = None

            # self.counter += 1


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