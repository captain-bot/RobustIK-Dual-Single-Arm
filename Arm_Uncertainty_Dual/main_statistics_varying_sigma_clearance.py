import ikModuleLeft
import ikModuleRight
import robot_parameters
import kinematic_utilities
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import time
import animate_assembly


"""
Input:
1. Best and worst IK solution pair
2. sigma_list: set of joint error standard deviation values
3. hole_radius_list: set of hole radius values

Output:
1. Surface plot of success rates. In X and Y we vary the clearance and joint noise respectively and in Z we get 
associated success rate.

Warning: If you want to change success strategy, change the condition of incrementing "failure_count". 
"""


# # Function to get noisy solution from a pure solution
# def get_noisy_solution_both(sol_l, sol_r, sig_val=0.0035):
#     noise = np.random.normal(0, sig_val, len(sol_l))
#     sol_noisy_l, sol_noisy_r = [], []
#     for indx in range(7):
#         sol_noisy_l.append(sol_l[indx] + noise[indx])
#         sol_noisy_r.append(sol_r[indx] + noise[indx])
#     return sol_noisy_l, sol_noisy_r


def visualize_all(T_base2leftgripper, T_base2rightgripper, T_base2peg, T_base2hole, dist, P_hole2pegfinal):
    # See the animation
    animate_assembly.view_animation(T_base2leftgripper, T_base2rightgripper, T_base2peg, T_base2hole, dist)

    # Plot hole circle
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.add_patch(plt.Circle((0, 0), robot_parameters.rad_hole, color='b', alpha=0.3, label="hole c/s"))
    ax.add_patch(plt.Circle((P_hole2pegfinal[0], P_hole2pegfinal[1]), robot_parameters.rad_peg, color='r', alpha=0.3,
                            label="peg c/s"))
    plt.axis("square")
    plt.xlabel("x_hole")
    plt.ylabel("y_hole")
    plt.title("View of peg and hole cross-sections in Hole-frame")
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.0)
    plt.show()
    # plt.waitforbuttonpress(0) # this will wait for indefinite time
    # plt.close()
    # plt.close(fig)


#########################################################################
########################## PART 0 #######################################
##########     Define all user defined parameters here    ###############
#########################################################################
# For 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# IK-Fast correction length
correction_length = 0.067

# Select maximum number of trials
num_trial = 10000

# Visualize
want_visual = False

# Generate grid with sigma and hole radius
sigma_list = [0.0020+0.0002*i for i in range(15)]
hole_radius_list = [0.027-0.0005*i for i in range(10)]
radial_clearance_list = [i - robot_parameters.rad_peg for i in hole_radius_list]
radial_clearance_grid, sig_val_grid = np.meshgrid(radial_clearance_list, sigma_list)

start_time = time.time()
for i in range(1, 2):
    if i == 0:
        # Solution pair: Best pair1
        left_js = [-0.361758, 0.320735, -2.99435, 0.572053, 1.27946, 1.93275, -0.493638]
        right_js = [0.493515, 0.550733, 2.88105, 1.20904, -1.36702, 1.55162, 0.838913]
    else:
        # worst pair1
        left_js = [-0.120334, 0.0840508, -1.98086, 0.50692, 0.323614, 1.80815, -0.346687]
        right_js = [0.278199, -0.710397, 0.709628, 1.20355, -2.08902, -1.33616, 3.04857]
    success_list = np.zeros(radial_clearance_grid.size)
    loop_count = 0
    for clearance_val, sigma_val in zip(list(radial_clearance_grid.flatten()), list(sig_val_grid.flatten())):
        # Compute available clearance
        available_clearance = clearance_val
        failure_count = 0
        for _ in range(num_trial):
            ###########################################################################
            ########################### PART 1 ########################################
            ################ Generate noisy solution from pure ones ###################
            ###########################################################################
            left_js_noisy, right_js_noisy = kinematic_utilities.get_noisy_solution_both(left_js, right_js, sig_val=sigma_val)

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
            if required_radial_clearance - available_clearance > 0.002:
                failure_count += 1
                # visualize_all(T_base2leftgripper, T_base2rightgripper, T_base2peg, T_base2hole, dist, P_hole2pegfinal)

        # Print total number of success
        num_success = (num_trial - failure_count)*100/num_trial
        print("Total success count: %2.4f percent" % num_success)
        success_list[loop_count] = num_success
        loop_count += 1
    print("Time elapsed: %2.6f" % (time.time() - start_time))
    success_grid = success_list.reshape(radial_clearance_grid.shape)

    # 3D Plot
    # ax.plot_surface(hole_radius_grid, sig_val_grid, success_grid, cmap='viridis', edgecolor='none')
    ax.plot_surface(radial_clearance_grid, sig_val_grid, success_grid, cmap=cm.coolwarm, linewidth=0, antialiased=False)

ax.set_xlabel('clearance [m]')
ax.set_ylabel(r'$\sigma$')
ax.set_zlabel('Success rate (%)')
ax.set_title('Change in success rate')
ax.view_init(30, 160)
plt.show()
