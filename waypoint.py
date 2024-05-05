import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
from autolab.core import RigidTransform
from frankapy import FrankaArm
from IKF import inverse_kinematics

class RobotState:
    RESET = 0
    MOVE_TO_BOOK = 1
    PICK_BOOK = 2
    LIFT_BOOK = 3
    ORIENT_BOOK = 4
    PLACE_BOOK = 5

current_state = RobotState.RESET
book_pose = None
books_queue = []  # Queue to store book poses

def move_to_reset_position(fa):
    # Move the robot to the reset position
    # reset_joint_angles = [0.0, -0.7, 0.0, -2.15, 0.0, 1.57, 0.7]
    reset_joint_angles = [ 0.05644431, -0.03321833 , 0.01825161, -1.4768081 , -0.05127762 , 1.48026603, 0.8659702 ]
    fa.goto_joints(reset_joint_angles)

def move_to_safe_distance_above_book(fa, book_pose):
    # Calculate safe distance above the book
    safe_distance = 0.1  # need changes
    desired_position = book_pose.translation + np.array([0, 0, safe_distance])

    # Move to the calculated position
    desired_pose = RigidTransform(translation=desired_position, from_frame='world', to_frame='franka_tool')
    fa.goto_gripper(0.5) # TBD based on the closeness of other books
    fa.goto_pose(desired_pose)

def pick_book(fa, gripper_width):
    # Adjust gripper width
    fa.goto_gripper(gripper_width)
    
    # Close gripper to pick the book
    fa.close_gripper()

def lift_book(fa, lift_height):
    # Lift the book vertically to a certain height
    current_pose = fa.get_pose()
    desired_pose = current_pose * RigidTransform(translation=np.array([0, 0, lift_height]), from_frame='franka_tool', to_frame='franka_tool')
    fa.goto_pose(desired_pose)

def orient_book(fa):
    # Rotate the book 90 degrees
    current_pose = fa.get_pose()
    rotation_matrix = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    rotated_pose = RigidTransform(rotation=rotation_matrix, from_frame='franka_tool', to_frame='franka_tool')
    desired_pose = current_pose * rotated_pose
    fa.goto_pose(desired_pose)

def place_book(fa, desired_location):
    # Move to the desired location to place the book
    # can be completed later, as of now just lift book

    pass

def book_pose_callback(msg):
    global current_state, book_pose
    # Convert ROS PoseStamped message to RigidTransform
    translation = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    rotation = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    book_pose = RigidTransform(rotation=rotation, translation=translation, from_frame='world', to_frame='franka_tool')

    ''' TODO: This will create a problem '''
    # Transition to the next state
    if current_state == RobotState.RESET:
        current_state = RobotState.MOVE_TO_BOOK

def main():
    global current_state, book_pose, books_queue
    # Initialize ROS node
    rospy.init_node('book_pickup_node')

    # Create FrankaArm instance
    fa = FrankaArm()

    # Reset joints
    move_to_reset_position(fa)

    # Subscribe to book pose
    rospy.Subscriber("book_pose", PoseStamped, book_pose_callback)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        if current_state == RobotState.MOVE_TO_BOOK:
            if book_pose is not None:
                move_to_safe_distance_above_book(fa, book_pose)
                current_state = RobotState.PICK_BOOK

        elif current_state == RobotState.PICK_BOOK:
            # Assuming gripper width and lift height are fixed
            gripper_width = 0.03
            lift_height = 0.2
            pick_book(fa, gripper_width)
            lift_book(fa, lift_height)
            current_state = RobotState.ORIENT_BOOK

        elif current_state == RobotState.ORIENT_BOOK:
            orient_book(fa)
            current_state = RobotState.PLACE_BOOK

        elif current_state == RobotState.PLACE_BOOK:
            # Assuming desired location is fixed for now
            desired_location = RigidTransform(translation=np.array([0.5, -0.5, 0.5]), from_frame='world', to_frame='franka_tool')
            place_book(fa, desired_location)
            current_state = RobotState.RESET
            book_pose = None  # Reset book pose

        rate.sleep()

if __name__ == "__main__":
    main()


'''########
point 1
pose: 
Tra: [ 0.48777115 -0.03213804  0.12531104]
 Rot: [[ 0.99905414 -0.00309281 -0.04315146]
 [-0.00212477 -0.99973582  0.02246161]
 [-0.04320953 -0.02234868 -0.99881601]]
 Qtn: [-0.01120593  0.9997007  -0.00130479 -0.02159671]
 from franka_tool to world

 Goal 1:
 pose: 
Tra: [0.19859332 0.56875431 0.44984054]
 Rot: [[ 0.02535552 -0.99860546 -0.04609787]
 [-0.02438276 -0.04671632  0.99861054]
 [-0.99937147 -0.0241963  -0.02553376]]
 Qtn: [-0.48813559  0.52383337 -0.48822215 -0.49895129]
 from franka_tool to world

'''