import ikModuleLeft
import ikModuleRight
import robot_parameters
import kinematic_utilities
import numpy as np
import matplotlib.pyplot as plt
import time
import animate_assembly_square

"""
This code takes in a pair of inverse kinematics solution one for left and another for right arm
and returns success rate by simulating peg-in-hole assembly.

Input (required): left_js, right_js
Input (optional): err_sig => joint error standard deviation(default 0.0035),
                num_trial => simulation sample size(default 10000)
                want_visualize => True to see animation (default True)
                outcome_print => True to see prints of relevant transforms (default True)

Warning: all robot parameters, including peg and hole lengths come from robot_parameters.py file
Warning: success rates are dependend on hole radius for fixed peg radius
Warning: success strategy: if required_clearance - available_clearance < 0.002, we consider it as
         a success and failure otherwise
"""


# This function prints relevant transforms
def func_print_trans(T_leftarmmount2leftgripper, T_base2leftgripper, T_base2peg, T_rightarmmount2rightgripper, T_base2rightgripper):
    print("\nT_leftarmmount2leftgripper: ")
    print(T_leftarmmount2leftgripper)

    print("\nT_base2leftgripper: ")
    print(T_base2leftgripper)

    print("\nT_base2peg: ")
    print(T_base2peg)

    print("\nT_rightarmmount2rightgripper: ")
    print(T_rightarmmount2rightgripper)

    print("\nT_base2rightgripper: ")
    print(T_base2rightgripper)

    print("\nT_base2hole: ")
    print(T_base2hole)


#########################################################################
########################## PART 0 #######################################
##########     Define all user defined parameters here    ###############
#########################################################################
# # Best pair1
left_js = [-0.361758, 0.320735, -2.99435, 0.572053, 1.27946, 1.93275, -0.493638]
right_js = [0.493515, 0.550733, 2.88105, 1.20904, -1.36702, 1.55162, 0.838913]

# # worst pair1
# left_js = [-0.120334, 0.0840508, -1.98086, 0.50692, 0.323614, 1.80815, -0.346687]
# right_js = [0.278199, -0.710397, 0.709628, 1.20355, -2.08902, -1.33616, 3.04857]

# # Best pair2
# left_js = [-0.21943, -0.559425, -0.313603, 0.848202, -1.24768, 1.94008, 0.045175]
# right_js = [0.480029, -0.927794, 0.343986, 1.43293, -1.78396, -1.43877, 2.88986]

# # worst pair2
# left_js = [0.0158911, 0.214207, -2.13239, 0.793455, 0.471557, 1.73486, -0.446869]
# right_js = [0.114549, -0.676611, 1.00135, 1.39467, -2.23933, -1.14559, 3.0349]

# Set joint error parameter
err_sig = 0.0035

# Select maximum number of trials
num_trial = 1

# Visualize
want_visual = True

# Print outcome
outcome_print = True

# Compute available clearance
available_clearance = (robot_parameters.hole_width - robot_parameters.peg_width)/2

# IK-Fast correction length
correction_length = 0.067

failure_count = 0
start_time = time.time()
max_x_axis_err = 0.0001
for _ in range(num_trial):
    ###########################################################################
    ########################### PART 1 ########################################
    ################ Generate noisy solution from pure ones ###################
    ###########################################################################
    left_js_noisy, right_js_noisy = kinematic_utilities.get_noisy_solution_both(left_js, right_js, sig_val=err_sig)
    # left_js_noisy, right_js_noisy = left_js, right_js

    ###########################################################################
    ########################### PART 2 ########################################
    ##    Compute Peg Pose wrt Base Frame using Left arm joint solution      ##
    ###########################################################################
    # Compute gripper frame wrt base frame
    T_leftarmmount2leftgripper = np.vstack((ikModuleLeft.compFK(left_js_noisy), np.array([0, 0, 0, 1])))
    T_leftarmmount2leftgripper[:3, 3] += correction_length * T_leftarmmount2leftgripper[:3, 2]
    T_base2leftgripper = np.dot(robot_parameters.T_base2leftarmmount, T_leftarmmount2leftgripper)


    # Compute peg frame wrt base frame
    T_base2peg = T_base2leftgripper.copy()
    T_base2peg[:3, 3] += robot_parameters.l_peg*T_base2peg[:3, 2]

    ###########################################################################
    ########################### PART 3 ########################################
    ##    Compute Hole Pose wrt Base Frame using Right arm joint solution    ##
    ###########################################################################
    # Compute gripper frame wrt base frame
    T_rightarmmount2rightgripper = np.vstack((ikModuleRight.compFK(right_js_noisy), np.array([0, 0, 0, 1])))
    T_rightarmmount2rightgripper[:3, 3] += correction_length * T_rightarmmount2rightgripper[:3, 2]
    T_base2rightgripper = np.dot(robot_parameters.T_base2rightarmmount, T_rightarmmount2rightgripper)

    # Compute peg frame wrt base frame
    T_base2hole = T_base2rightgripper.copy()
    T_base2hole[:3, 3] += robot_parameters.l_hole*T_base2hole[:3, 2]

    ############################################################################
    ############################ PART 4 ########################################
    ################   Move peg along its z-axis towards hole ##################
    ############################################################################
    dist = kinematic_utilities.norm(T_base2peg[:3, 3] - T_base2hole[:3, 3])
    print("Distance between peg and hole frames before motion starts: %2.6f" % dist)

    # Update the position of peg frame after sliding it along z-axis by dist
    peg_final_position = T_base2peg[:3, 3] + dist * T_base2peg[:3, 2]
    P_hole2pegfinal = np.dot(T_base2hole[:3, :3].T, peg_final_position - T_base2hole[:3, 3])
    print("P_hole2pegfinal: ")
    print(P_hole2pegfinal)

    # Find orientation error between the X-axes of Peg and Hole
    x_axis_error = np.arccos(np.dot(T_base2hole[:3, 0], T_base2peg[:3, 0]))
    x_axis_err_deg = x_axis_error*180/np.pi

    if x_axis_err_deg > max_x_axis_err:
        max_x_axis_err = x_axis_err_deg

    ############################################################################
    ############################# PART 5 #######################################
    ########### Visualize peg and hole cross sections in hole frame XY plane ###
    ############################################################################
    if outcome_print:
        func_print_trans(T_leftarmmount2leftgripper, T_base2leftgripper, T_base2peg, T_rightarmmount2rightgripper,
                         T_base2rightgripper)

    if want_visual:
        animate_assembly_square.view_animation_square(T_base2leftgripper, T_base2rightgripper, T_base2peg, T_base2hole, dist)

    # Check success
    required_radial_clearance = kinematic_utilities.norm(P_hole2pegfinal[:2])
    print("Difference between required and available clearance: %2.4f" % (required_radial_clearance - available_clearance))
    print("Angular error between x-axes of peg and hole: %2.4f" % x_axis_err_deg)
    if (required_radial_clearance - available_clearance > 0.002) or (x_axis_err_deg > 1.6):
    # if (required_radial_clearance - available_clearance > 0.002):
        failure_count += 1
        print("Peg and hole are in collision")
        print("Distance between peg and hole centers: %2.6f" % required_radial_clearance)

# Print total number of success
print("Total success count: %2.4f percent" % ((num_trial - failure_count)*100/num_trial))
print("Maximum x-axis error: %2.4f degree" % max_x_axis_err)
print("Time elapsed: %2.6f" % (time.time() - start_time))
