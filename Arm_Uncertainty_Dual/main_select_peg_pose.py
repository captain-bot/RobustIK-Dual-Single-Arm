"""
1. This piece of code takes T_base2rightgripper as input and output T_rightarmmount2rightgripper
and T_leftarmmount2leftgripper poses which are useful to compute IK solutions from IK-Fast.
2. Further this piece of code shows animation to confirm that correct poses of peg and hole have
been computed.
Date: July 16th, 2019
Author: Anirban Sinha, State University of New York, Stony Brook
"""
import numpy as np
import kinematic_utilities
import robot_parameters
import animate_assembly

###########################################################
###################### User Input #########################
###########################################################
# Pose of right_gripper frame wrt base frame
# # Test case: 1
# p_base2rightgripper = np.array([0.786, -0.003, 0.459])
# R_base2rightgripper = kinematic_utilities.unitquat2rotm([-0.057, 0.093, 0.717, 0.688])

# Test case: 2
p_base2rightgripper = np.array([0.746, 0.023, 0.459])
R_base2rightgripper = kinematic_utilities.unitquat2rotm([-0.057, 0.093, 0.717, 0.688])
T_base2rightgripper = np.vstack((np.hstack((R_base2rightgripper, p_base2rightgripper.reshape(3, 1))), np.array([0, 0, 0, 1])))
print("Given::T_base2rightgripper: ")
print(T_base2rightgripper, "\n")

############################################################
################## Compute Transformations #################
############################################################
# Compute transformation of hole frame wrt base frame
p_base2hole = p_base2rightgripper + robot_parameters.l_hole*R_base2rightgripper[:3, 2]
T_base2hole = np.vstack((np.hstack((R_base2rightgripper.copy(), p_base2hole.reshape(3, 1))), np.array([0, 0, 0, 1])))
print("T_base2hole: ")
print(T_base2hole, "\n")

# Compute transformation of peg frame wrt base frame
p_base2peg = p_base2rightgripper + 3*robot_parameters.l_hole*R_base2rightgripper[:3, 2]
R_base2peg = np.hstack((R_base2rightgripper[:3, 0].reshape(3, 1), -R_base2rightgripper[:3, 1].reshape(3, 1),
                        -R_base2rightgripper[:3, 2].reshape(3, 1)))
T_base2peg = np.vstack((np.hstack((R_base2peg.copy(), p_base2peg.reshape(3, 1))), np.array([0, 0, 0, 1])))
print("T_base2peg: ")
print(T_base2peg, "\n")

# Compute transformation of left_gripper frame wrt base frame
p_base2leftgripper = p_base2peg - robot_parameters.l_peg*R_base2peg[:3, 2]
T_base2leftgripper = np.vstack((np.hstack((R_base2peg.copy(), p_base2leftgripper.reshape(3, 1))),
                                np.array([0, 0, 0, 1])))
print("T_base2leftgripper: ")
print(T_base2leftgripper, "\n")

# Compute transformation of left_gripper frame wrt left_arm_mount frame
T_leftarmmount2leftgripper = np.dot(kinematic_utilities.inv(robot_parameters.T_base2leftarmmount), T_base2leftgripper)
print("T_leftarmmount2leftgripper: ")
print(T_leftarmmount2leftgripper, "\n")

# Compute transformation of right_gripper frame wrt right_arm_mount frame
T_rightarmmount2rightgripper = np.dot(kinematic_utilities.inv(robot_parameters.T_base2rightarmmount),
                                      T_base2rightgripper)
print("T_rightarmmount2rightgripper: ")
print(T_rightarmmount2rightgripper, "\n")

###################################################################################################
######## Transformations of left and right grippers wrt corresponding arm_mount frames ############
###################################################################################################
print("Following can be used for IK-Fast input for computing IK-solutions")
print("pd_l: ", T_leftarmmount2leftgripper[:3, 3])
print("qd_l: ", kinematic_utilities.rotm2quat(T_leftarmmount2leftgripper[:3, :3]))

print("pd_r: ", T_rightarmmount2rightgripper[:3, 3])
print("qd_r: ", kinematic_utilities.rotm2quat(T_rightarmmount2rightgripper[:3, :3]))

###################################################################################################
######################## Visualize ################################################################
###################################################################################################
dist = kinematic_utilities.norm(T_base2leftgripper[:3, 3] - T_base2rightgripper[:3, 3])
animate_assembly.view_animation(T_base2leftgripper, T_base2rightgripper, T_base2peg, T_base2hole, dist)
