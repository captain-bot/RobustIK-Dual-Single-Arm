import kinematic_utilities
import robot_parameters_sl
import numpy as np


"""
This piece of code returns left-gripper pose with respect to left-arm-mount frame
given left-gripper pose with respect to base frame of Baxter robot.
"""


# User input
p_base2leftgripper = np.array([0.701, 0.101, 0.380])
R_base2leftgripper = kinematic_utilities.unitquat2rotm([0.6839, 0.7174, 0.0799, -0.1064])
T_base2leftgripper = np.vstack((np.hstack((R_base2leftgripper, p_base2leftgripper.reshape(3, 1))), np.array([0, 0, 0, 1])))
print("Given::T_base2rightgripper: ")
print(T_base2leftgripper, "\n")

# Compute transformation of left_gripper frame wrt left_arm_mount frame
T_leftarmmount2leftgripper = np.dot(kinematic_utilities.inv(robot_parameters_sl.T_base2leftarmmount), T_base2leftgripper)
print("T_leftarmmount2leftgripper: ")
print(T_leftarmmount2leftgripper, "\n")

# For IK-Fast input
print("Following can be used for IK-Fast input for computing IK-solutions")
print("pd_l: ", T_leftarmmount2leftgripper[:3, 3])
print("qd_l: ", kinematic_utilities.rotm2quat(T_leftarmmount2leftgripper[:3, :3]))
