from frankapy import FrankaArm
import numpy as np
import rospy
from geometry_msgs.msg import Pose
from autolab_core import RigidTransform

if __name__ == "__main__":
    print("started..")
    fa = FrankaArm()
    # fa.reset_joints()
    # print(fa.get_gripper_width())
    # fa.goto_gripper(0.03)
    fa.close_gripper()
    print("closed gripper..")

    # T_ee_world = fa.get_pose()
    # joints = fa.get_joints()

    # gripper_width = fa.get_gripper_width()
    # force_torque = fa.get_ee_force_torque()

    # print(f"translation: {T_ee_world.translation}")
    # print(f"rot: {T_ee_world.quaternion}")

    # print(f"gripper width: {gripper_width}")


    # # fa.goto_joints([0.0, -0.7, 0.0, -2.15, 0.0, 1.57, 0.7])
    # fa.goto_joints([ 0.04956721, 0.54953135, 0.27469185, -2.08371178, -0.17563049, 2.73373699, 1.26298713])
    # fa.close_gripper()
    
    # fa.open_gripper()
    # fa.goto_gripper(0.03)

    des_pose = RigidTransform(rotation=np.array([
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0]]),
        translation=np.array([0.3, -0.2, 0.4]),
        from_frame = 'franka_tool', to_frame='world'
    )
    print("psoe....")

    fa.goto_pose(des_pose, use_impedance=False)
