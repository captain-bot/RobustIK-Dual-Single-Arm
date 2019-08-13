import ikModuleLeft
import ikModuleRight
import robot_parameters_sl
import kinematic_utilities
import numpy as np
import matplotlib.pyplot as plt
import time
import animate_assembly_sl


# Function to get noisy solution from a pure solution
def get_noisy_solution_single(sol_l, sig_val=0.0035):
    noise = np.random.normal(0, sig_val, len(sol_l))
    sol_noisy_l = []
    for indx in range(7):
        sol_noisy_l.append(sol_l[indx] + noise[indx])
    return sol_noisy_l


def visualize_all(T_base2leftgripper, T_base2rightgripper, T_base2peg, T_base2hole, dist, P_hole2pegfinal):
    # See the animation
    animate_assembly_sl.view_animation(T_base2leftgripper, T_base2rightgripper, T_base2peg, T_base2hole, dist)

    # Plot hole circle
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.add_patch(plt.Circle((0, 0), robot_parameters_sl.rad_hole, color='b', alpha=0.3, label="hole c/s"))
    ax.add_patch(plt.Circle((P_hole2pegfinal[0], P_hole2pegfinal[1]), robot_parameters_sl.rad_peg, color='r', alpha=0.3,
                            label="peg c/s"))
    plt.axis("square")
    plt.xlabel("x_hole")
    plt.ylabel("y_hole")
    plt.title("View of peg and hole cross-sections in Hole-frame")
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.0)
    plt.show()


#########################################################################
########################## PART 0 #######################################
##########     Define all user defined parameters here    ###############
#########################################################################
# Pose 1
# left_js = [-0.890312, -0.305493, 0.722316, 0.982305, -2.02034, 2.07934, -2.86657]  # best
# left_js = [-0.0412668, 0.205703, -1.6246, 0.904117, 0.226834, 1.44901, 2.95739]  # worst
# Pose 2
# left_js = [-0.93577, -0.93764, 0.62227, 1.74906, -1.50413, 2.07934, -2.5891]  # best
# left_js = [0.277065, -0.504565, -1.21951, 1.67051, -0.598543, 1.19981, -3.04102]  # worst
# Pose 3
# left_js = [-0.737564, -0.557361, 0.498008, 1.56846, -1.45752, 2.07934, -2.33842]  # best
# left_js = [0.382401, 0.494503, -1.69693, 1.47334, 0.561846, 1.36106, 2.80229]  # worst
# Pose 4
# left_js = [-0.943977, -0.352305, 0.642545, 1.29687, -1.84105, 2.07934, -2.56546]  # best
# left_js = [0.0749175, 0.585803, -1.88147, 1.22042, 0.705758, 1.36839, 2.81761]  # worst
# Pose 5
# left_js = [-0.772714, 0.744743, 2.69684, 1.40361, 1.64951, 2.07934, 2.23653]  # best
left_js = [0.0500522, -0.518814, -0.948977, 1.33997, -0.719901, 1.4197, -3.01948]  # worst

# IK-Fast correction length
correction_length = 0.067

# Select maximum number of trials
num_trial = 10000

# Visualize
want_visual = False

# Compute available clearance
available_clearance = robot_parameters_sl.rad_hole - robot_parameters_sl.rad_peg

failure_count = 0
start_time = time.time()
for trial in range(num_trial):
    ###########################################################################
    ########################### PART 1 ########################################
    ################ Generate noisy solution from pure ones ###################
    ###########################################################################
    if trial == 0:
        left_js_noisy = left_js
    else:
        left_js_noisy = get_noisy_solution_single(left_js, sig_val=0.0035)

    ###########################################################################
    ########################### PART 2 ########################################
    ##    Compute Peg Pose wrt Base Frame using Left arm joint solution      ##
    ###########################################################################
    # Compute gripper frame wrt base frame
    T_leftarmmount2leftgripper = np.vstack((ikModuleLeft.compFK(left_js_noisy), np.array([0, 0, 0, 1])))
    T_leftarmmount2leftgripper[:3, 3] += correction_length * T_leftarmmount2leftgripper[:3, 2]
    T_base2leftgripper = np.dot(robot_parameters_sl.T_base2leftarmmount, T_leftarmmount2leftgripper)
    print("\nT_leftarmmount2leftgripper: ")
    print(T_leftarmmount2leftgripper)

    print("\nT_base2leftgripper: ")
    print(T_base2leftgripper)

    # Compute peg frame wrt base frame
    T_base2peg = T_base2leftgripper.copy()
    T_base2peg[:3, 3] += robot_parameters_sl.l_peg*T_base2peg[:3, 2]
    print("\nT_base2peg: ")
    print(T_base2peg)

    if trial == 0:
        ###########################################################################
        ########################### PART 3 ########################################
        ##    Compute Hole Pose wrt Base Frame using Right arm joint solution    ##
        ###########################################################################
        # Compute hole-pose wrt base frame
        p_base2hole = T_base2peg[:3, 3] + robot_parameters_sl.l_peg*T_base2peg[:3, 2]
        R_base2hole = np.hstack(((T_base2peg[:3, 0].reshape(3, 1), -T_base2peg[:3, 1].reshape(3, 1),
                            -T_base2peg[:3, 2].reshape(3, 1))))
        T_base2hole = np.vstack((np.hstack((R_base2hole.copy(), p_base2hole.reshape(3, 1))), np.array([0, 0, 0, 1])))
        print("\nT_base2hole: ")
        print(T_base2hole)

        # Compute right_gripper pose wrt base frame
        p_base2rightgripper = T_base2hole[:3, 3] - robot_parameters_sl.l_hole * R_base2hole[:3, 2]
        R_base2rightgripper = R_base2hole.copy()
        T_base2rightgripper = np.vstack((np.hstack((R_base2rightgripper.copy(), p_base2rightgripper.reshape(3, 1))),
                                         np.array([0, 0, 0, 1])))
        print("\nT_base2rightgripper: ")
        print(T_base2rightgripper)

    ############################################################################
    ############################ PART 4 ########################################
    ################   Move peg along its z-axis towards hole ##################
    ############################################################################
    dist = kinematic_utilities.norm(T_base2peg[:3, 3] - T_base2hole[:3, 3])
    print("Distance between peg and hole frames before motion starts: %2.6f" % dist)

    # Update the position of peg frame after sliding it along z-axis by dist
    peg_final_position = T_base2peg[:3, 3] + dist * T_base2peg[:3, 2]
    P_hole2pegfinal = np.dot(T_base2hole[:3, :3].T, peg_final_position - T_base2hole[:3, 3])

    ############################################################################
    ############################# PART 5 #######################################
    ########### Visualize peg and hole cross sections in hole frame XY plane ###
    ############################################################################
    if want_visual:
        visualize_all(T_base2leftgripper, T_base2rightgripper, T_base2peg, T_base2hole, dist, P_hole2pegfinal)

    # Check success
    required_radial_clearance = kinematic_utilities.norm(P_hole2pegfinal[:2])
    print("required clearance: ", required_radial_clearance)
    if required_radial_clearance - available_clearance > 1e-3:
        failure_count += 1
        # visualize_all(T_base2leftgripper, T_base2rightgripper, T_base2peg, T_base2hole, dist, P_hole2pegfinal)
        print("Peg and hole are in collision")
        print("Distance between peg and hole centers: %2.6f" % required_radial_clearance)

# Print total number of success
print("Total success count: %2.4f percent" % ((num_trial - failure_count)*100/num_trial))
print("Time elapsed: %2.6f" % (time.time() - start_time))
