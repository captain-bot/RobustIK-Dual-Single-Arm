"""
This piece of code computes IK solutions for left and right arms given the corresponding gripper poses.
Please note that, the gripper poses should correspond to "standard narrow" jaws. The ik solutions of
left and right arms will be written in files named "ik_sol_config1.txt" and "ik_sol_config2.txt" respectively.
"""

from kinematic_utilities import *
import ikModuleLeft
import ikModuleRight
import numpy as np

# # Desired pose of left_gripper frame wrt left_arm_mount frame
pd_l = np.array([0.528, -0.562, 0.340])
qd_l = np.array([0.360, -0.210, -0.657,  0.627])

# Desired pose of right_gripper frame wrt right_arm_mount frame
pd_r = np.array([0.385,  0.692, 0.350])
qd_r = np.array([0.316,  0.188, -0.698, -0.614])

# Get the inverse kinematics solutions
print("Start of computing IK solutions for left-arm")
get_iksol(pd_l, qd_l, adjust_len=0.067, side="left", config_num=1)
print("\n\n\n\n")

print("Start of computing IK solutions for right-arm")
get_iksol(pd_r, qd_r, adjust_len=0.067, side="right", config_num=2)

# # Cross validate forward kinematics of left arm
print("Warning: You need to type left and right arm solutions manually. Other wise following result has no meaning")
left_sol = [-0.514, 0.312, 2.837, 0.580, 1.692, 2.028, -0.507]
left_FK = ikModuleLeft.compFK(left_sol)
left_FK[:3, 3] += 0.067*left_FK[:3, 2]
print("\n => Left arm position: ", left_FK[:3, 3])
print("=> Left arm orientation(quaternion): ", rotm2quat(left_FK[:3, :3]))

# # Cross validate forward kinematics of right arm
print("Warning: You need to type left and right arm solutions manually. Other wise following result has no meaning")
right_sol = [0.287, -0.717, 0.647, 1.179, 1.082, 1.390, -0.093]
right_FK = ikModuleRight.compFK(right_sol)
right_FK[:3, 3] += 0.067*right_FK[:3, 2]
print("\n => Right arm position: ", right_FK[:3, 3])
print("=> Right arm orientation(quaternion): ", rotm2quat(right_FK[:3, :3]))
