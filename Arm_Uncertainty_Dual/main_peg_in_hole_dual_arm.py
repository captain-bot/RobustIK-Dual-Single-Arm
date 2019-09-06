from kinematic_utilities import *
from robot_parameters import *
import pickle

"""
This code takes in desired position and orientation of peg and hole end-effector 
frames with respect to the corresponding arm_mount frames and returns pairs of indices
of left and right arm IK solution list based on robust and worst objective value for peg
in hole assembly. On completion of the code execution, therelevant variable values are 
written in binary file named "dual_arm_assembly_file"

Warning: make sure that quaternions are in [q0, q1, q2, q3] format
Warning: attached Baxter's end-effector should be standard narrow
Warning: (pd_l, qd_l) is left end-effector pose wrt left_arm_mount frame
Warning: (pd_r, qd_r) is right end-effector pose wrt right_arm_mount frame
Warning: output IK-pair indices are counted from 0
"""

# # Desired pose of left_gripper frame wrt left_arm_mount frame
# # pd_l = np.array([0.222, -0.507, 0.182])
# # qd_l = np.array([-0.282, 0.209, 0.679, -0.644])
# pd_l = np.array([0.338, -0.590, 0.199])
# qd_l = np.array([0.161, -0.296, -0.655, 0.676])
#
# # Desired pose of right_gripper frame wrt right_arm_mount frame
# pd_r = np.array([0.350, 0.584, 0.193])
# qd_r = np.array([-0.239, -0.355, 0.576, 0.697])

# Working solution (may be related to the paper data)
# # Desired pose of left_gripper frame wrt left_arm_mount frame
pd_l = np.array([0.528, -0.562, 0.340])
qd_l = np.array([0.360, -0.210, -0.657,  0.627])

# Desired pose of right_gripper frame wrt right_arm_mount frame
pd_r = np.array([0.385,  0.692, 0.350])
qd_r = np.array([0.316,  0.188, -0.698, -0.614])

# # Currently testing
# pd_l = np.array([0.518, -0.514, 0.340])
# qd_l = np.array([0.360, -0.210, -0.657,  0.627])
# pd_r = np.array([0.338, 0.681, 0.350])
# qd_r = np.array([0.316, 0.188, -0.698, -0.614])

# Get the inverse kinematics solutions
get_iksol(pd_l, qd_l, adjust_len=0.067, side="left", config_num=1)
get_iksol(pd_r, qd_r, adjust_len=0.067, side="right", config_num=2)

# Read the solution files
left_arm_sols, right_arm_sols = [], []
file = open("ik_sol_config1.txt", "r")
for line in file:
    th = list()
    for ele in line.split(","):
        th.append(float(ele))
    left_arm_sols.append(th)
print(left_arm_sols)

file = open("ik_sol_config2.txt", "r")
for line in file:
    th = list()
    for ele in line.split(","):
        th.append(float(ele))
    right_arm_sols.append(th)
print(right_arm_sols)

# Start computing error for different IK-pair
assembly_err_list = []
ik_pair_list = []
wr_c = np.vstack((wr_l, wr_r))
qr_c = np.vstack((qr_l, qr_r))
for indx_l, js_l in enumerate(left_arm_sols):
    # Solve forward kinematics for left arm
    gst_l, transform_upto_joint_left = exp_direct_kin(gst0_l, wr_l, qr_l, js_l)

    # if indx_l > 50:
    #     break

    for indx_r, js_r in enumerate(right_arm_sols):
        # Solve forward kinematics for right arm
        gst_r, transform_upto_joint_right = exp_direct_kin(gst0_r, wr_r, qr_r, js_r)

        # Compute desired relative gripper poses
        g_relative_desired = np.dot(inv(gst_l), gst_r)
        pd = g_relative_desired[:3, 3]
        Rd = g_relative_desired[:3, :3]
        qd = rotm2quat(Rd)
        if qd is None:
            print("skipping")
            continue

        theta_c = js_l + js_r
        analytical_relative_jacobian, spatial_relative_jacobian = \
            relative_jac(gst0_l, gst0_r, gst_l, gst_r, transform_upto_joint_left, transform_upto_joint_right, wr_c,
                         qr_c, theta_c)

        # Separate position and orientation Jacobians
        jp = analytical_relative_jacobian[:3, :]
        jr = analytical_relative_jacobian[3:, :]

        # Compute position error bound
        err_p = position_err_bound(jp, c_val)
        print("\nComputed position error: %2.6f meters\n" % err_p)

        # Compute orientation error bound
        #  Make sure q = [W,X,Y,Z] or q = [q0, q1, q2, q3] format
        err_r, q_star = rotation_err_bound(jr, c_val, qd)
        print("Computed rotation error: %2.6f radians\n" % err_r)

        # Compute combined position and orientation error for dual-arm assembly
        R_a = unitquat2rotm(q_star)
        R_d = unitquat2rotm(qd)
        err_total = err_p + l_hole * norm(R_a[:3, 2] - R_d[:3, 2])
        print("Combined position and orientation error: %2.6f meters\n" % err_total)

        # Add it to error list
        assembly_err_list.append(err_total)
        ik_pair_list.append([indx_l, indx_r])
        print("====================================================")

min_index = assembly_err_list.index(min(assembly_err_list))
max_index = assembly_err_list.index(max(assembly_err_list))

print("Minimum error ik-pair: ", ik_pair_list[min_index])
print("Maximum error ik-pair: ", ik_pair_list[max_index])

print("Minimum error: %2.6f" % min(assembly_err_list))
print("Maximum error: %2.6f" % max(assembly_err_list))

# # Save data using pickle
# dbfile = open('dual_arm_assembly', 'ab')  # "ab" : append-binary
#
# save_dict = dict()
# save_dict["assembly_err_list"] = assembly_err_list
# save_dict["ik_pair_list"] = ik_pair_list
# save_dict["left_arm_sols"] = left_arm_sols
# save_dict["right_arm_sols"] = right_arm_sols
# save_dict["pd_l"] = pd_l
# save_dict["pd_r"] = pd_r
# save_dict["qd_l"] = qd_l
# save_dict["qd_r"] = qd_r
#
# pickle.dump(save_dict, dbfile)
# dbfile.close()


