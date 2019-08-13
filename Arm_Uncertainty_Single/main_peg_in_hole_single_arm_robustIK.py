from kinematic_utilities import *
from robot_parameters_sl import *

"""
This piece of code computes robust-IK solution for desired peg pose assuming hole pose is known perfectly.
This code assumes that the peg is held in left gripper and hole in the right.
1. Input: pd_l => desired position of left gripper wrt left_arm_mount frame
2. Output: Index of the joint solution set associated to robust-IK
Date: July 19th, 2019
Author: Anirban Sinha, State University of New York, Stony Brook, New York, USA
"""

# Input: position and orientation of left_gripper frame wrt left_arm_mount frame
pd_l = np.array([0.394, -0.562, 0.271])
qd_l = np.array([0.591, 0.693, -0.201, -0.360])

# Get the inverse kinematics solutions
get_iksol(pd_l, qd_l, adjust_len=0.067, side="left", config_num=1)

# Read the solution files
left_arm_sols = []
file = open("ik_sol_config1.txt", "r")
for line in file:
    th = list()
    for ele in line.split(","):
        th.append(float(ele))
    left_arm_sols.append(th)
print(left_arm_sols)

err_list = []
for indx_l, js_l in enumerate(left_arm_sols):
    # Solve forward kinematics for left arm
    gst_l, transform_upto_joint_left = exp_direct_kin(gst0_l, wr_l, qr_l, js_l)

    # Compute spatial jacobian
    jac_spatial, jac_analytical = velocity_direct_kin(gst0_l, wr_l, qr_l, js_l)

    # Separate position and orientation Jacobians
    jp = jac_analytical[:3, :]
    jr = jac_analytical[3:, :]

    # Compute position error bound
    err_p = position_err_bound(jp, c_val)
    print("\nComputed position error: %2.6f meters\n" % err_p)

    # Compute orientation error bound
    #  Make sure q = [W,X,Y,Z] or q = [q0, q1, q2, q3] format
    err_r, q_star = rotation_err_bound(jr, c_val, qd_l)
    print("Computed rotation error: %2.6f radians\n" % err_r)

    # Compute combined position and orientation error for dual-arm assembly
    R_a = unitquat2rotm(q_star)
    R_d = unitquat2rotm(qd_l)
    err_total = err_p + l_hole * norm(R_a[:3, 2] - R_d[:3, 2])
    print("Combined position and orientation error: %2.6f meters\n" % err_total)

    # Add it to error list
    err_list.append(err_total)

    print("====================================================")

min_index = err_list.index(min(err_list))
max_index = err_list.index(max(err_list))

print("minimum error solution index: %d" % min_index)
print("maximum error solution index: %d" % max_index)

print("Minimum error ik-pair: ", left_arm_sols[min_index])
print("Maximum error ik-pair: ", left_arm_sols[max_index])

print("Minimum error: %2.6f" % min(err_list))
print("Maximum error: %2.6f" % max(err_list))
