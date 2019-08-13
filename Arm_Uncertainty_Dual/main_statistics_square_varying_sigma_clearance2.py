import ikModuleLeft
import ikModuleRight
import robot_parameters
import kinematic_utilities
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import time
import animate_assembly_square
import collision_lib


#########################################################################
########################## PART 0 #######################################
##########     Define all user defined parameters here    ###############
#########################################################################
# # For 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# IK-Fast correction length
correction_length = 0.067

# Select maximum number of trials
num_trial = 10000

# Visualize
want_visual = False

# Generate grid with sigma and hole radius
sigma_list = [0.0020+0.0005*i for i in range(6)]
hole_width_list = [robot_parameters.hole_width-0.0005*i for i in range(10)]
clearance_list = [0.5*(i - robot_parameters.peg_width) for i in hole_width_list]
# clearance_grid, sig_val_grid = np.meshgrid(clearance_list, sigma_list)

start_time = time.time()
for sv in sigma_list:
    for i in range(0, 2):
        if i == 0:
            # Solution pair: Best pair1
            left_js = [-0.361758, 0.320735, -2.99435, 0.572053, 1.27946, 1.93275, -0.493638]
            right_js = [0.493515, 0.550733, 2.88105, 1.20904, -1.36702, 1.55162, 0.838913]
        else:
            # worst pair1
            left_js = [-0.120334, 0.0840508, -1.98086, 0.50692, 0.323614, 1.80815, -0.346687]
            right_js = [0.278199, -0.710397, 0.709628, 1.20355, -2.08902, -1.33616, 3.04857]

        success_list = [0]*len(hole_width_list)

        sim_count = 0
        for hw in hole_width_list:
            failure_count = 0
            for _ in range(num_trial):
                ###########################################################################
                ########################### PART 1 ########################################
                ################ Generate noisy solution from pure ones ###################
                ###########################################################################
                left_js_noisy, right_js_noisy = kinematic_utilities.get_noisy_solution_both(left_js, right_js,
                                                                                            sig_val=sv)

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
                T_base2peg[:3, 3] += robot_parameters.l_peg * T_base2peg[:3, 2]

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
                T_base2hole[:3, 3] += robot_parameters.l_hole * T_base2hole[:3, 2]

                ############################################################################
                ############################ PART 4 ########################################
                ################   Move peg along its z-axis towards hole ##################
                ############################################################################
                dist = kinematic_utilities.norm(T_base2peg[:3, 3] - T_base2hole[:3, 3])

                # Update the position of peg frame after sliding it along z-axis by dist
                peg_final_position = T_base2peg[:3, 3] + dist * T_base2peg[:3, 2]
                P_hole2pegfinal = np.dot(T_base2hole[:3, :3].T, peg_final_position - T_base2hole[:3, 3])
                R_hole2pegfinal = np.dot(T_base2hole[:3, :3].T, T_base2peg[:3, :3])

                ############################################################################
                ############################# PART 5 #######################################
                ########### Visualize peg and hole cross sections in hole frame XY plane ###
                ############################################################################
                if want_visual:
                    animate_assembly_square.visualize_all(T_base2leftgripper, T_base2rightgripper, T_base2peg,
                                                          T_base2hole, dist, P_hole2pegfinal, R_hole2pegfinal)

                # Check collision
                if collision_lib.collision_square(P_hole2pegfinal, R_hole2pegfinal, hole_width=hw):
                    print("Collision: True")
                    failure_count += 1
                else:
                    print("Collision: False")

            # Print total number of success
            num_success = (num_trial - failure_count) * 100 / num_trial
            print("Total success count: %2.4f percent" % num_success)
            success_list[sim_count] = num_success
            sim_count += 1

        # # Plot legend
        # if (i == 0) and (sv == 0.002) and (hw == robot_parameters.hole_width):
        #     ax.plot(clearance_list, [sv] * len(clearance_list), success_list, "b--", label=r"$\Theta^*$")
        #     continue
        # elif (i == 1) and (sv == 0.002) and (hw == robot_parameters.hole_width):
        #     ax.plot(clearance_list, [sv] * len(clearance_list), success_list, "r-.", label=r"$\Theta^-$")
        #     continue

        if i == 0:
            ax.plot(clearance_list, [sv]*len(clearance_list), success_list, color=(0.2, 0.2, 1.0), linestyle='--', marker='o')
        else:
            ax.plot(clearance_list, [sv]*len(clearance_list), success_list, color=(1.0, 0.2, 0.2), linestyle='-.', marker='o')

ax.set_xlabel("clearance [m]")
ax.set_ylabel(r"$\sigma$ [rad]")
ax.set_zlabel("Success rate percentage [%]")
# ax.legend()
plt.show()
