from kinematic_utilities import *
from robot_parameters import *
import pickle
from matplotlib import pyplot as plt
import time


# Load generated data
dbfile = open("dual_arm_assembly", "rb")
db = pickle.load(dbfile)

assembly_err_list = db["assembly_err_list"]
ik_pair_list = db["ik_pair_list"]
left_arm_sols = db["left_arm_sols"]
right_arm_sols = db["right_arm_sols"]
pd_l, qd_l = db["pd_l"], db["qd_l"]
pd_r, qd_r = db["pd_r"], db["qd_r"]

dbfile.close()

# Get IK-pair with minimum and maximum error
min_index = assembly_err_list.index(min(assembly_err_list))
max_index = assembly_err_list.index(max(assembly_err_list))

print("Minimum error ik-pair: ", ik_pair_list[min_index])
print("Maximum error ik-pair: ", ik_pair_list[max_index])

print("Minimum error: %2.6f" % min(assembly_err_list))
print("Maximum error: %2.6f" % max(assembly_err_list))

best_indices = ik_pair_list[min_index]
worst_indices = ik_pair_list[max_index]

print(left_arm_sols[best_indices[0]])
print(right_arm_sols[best_indices[1]])

print(left_arm_sols[worst_indices[0]])
print(right_arm_sols[worst_indices[1]])


def rel_hole_wrt_peg(js_l, js_r):
    # Solve forward kinematics
    gst_l, transform_upto_joint_left = exp_direct_kin(gst0_l, wr_l, qr_l, js_l)
    gst_r, transform_upto_joint_right = exp_direct_kin(gst0_r, wr_r, qr_r, js_r)
    # print("gst_l")
    # print(gst_l)
    # print("gst_r")
    # print(gst_r)

    # Get peg tip pose
    R_peg = gst_l[:3, :3]
    p_peg = gst_l[:3, 3] + l_peg * gst_l[:3, 2]
    pose_peg = np.hstack((R_peg, p_peg.reshape(3, 1)))
    pose_peg = np.vstack((pose_peg, np.array([0, 0, 0, 1])))
    print("Pose of the peg: ")
    print(pose_peg)
    # print(gst_l)

    # Get hole tip pose
    R_hole = gst_r[:3, :3]
    p_hole = gst_r[:3, 3] + l_hole * gst_r[:3, 2]
    pose_hole = np.hstack((R_hole, p_hole.reshape(3, 1)))
    pose_hole = np.vstack((pose_hole, np.array([0, 0, 0, 1])))
    print("Pose of the hole: ")
    print(pose_hole)
    # print(gst_r)

    # Compute desired relative pose of hole tip frame wrt peg tip frame
    relative_pose = np.dot(inv(pose_peg), pose_hole)
    return relative_pose


def get_noisy_solution(sol, mu_val=0, sig_val=0.0035):
    noise = np.random.normal(mu_val, sig_val, len(sol))
    sol_noisy = []
    for indx, ele in enumerate(sol):
        sol_noisy.append(ele + noise[indx])
    return sol_noisy


def get_noisy_solution_both(sol_l, sol_r):
    noise = np.random.normal(0, 0.0035, len(sol_l))
    sol_noisy_l, sol_noisy_r = [], []
    for indx in range(7):
        sol_noisy_l.append(sol_l[indx] + noise[indx])
        sol_noisy_r.append(sol_r[indx] + noise[indx])
    return sol_noisy_l, sol_noisy_r


# Compute success rate
def compute_success_rates(jl, jr, tol_set, sigma_value=0.0035, trial=100):
    # Sample size
    # trial = 10000
    success_rate_list = []
    for tol in tol_set:
        # Noise free
        # jl = left_arm_sols[best_indices[0]]
        # jr = right_arm_sols[best_indices[1]]
        desired_relative_pose = rel_hole_wrt_peg(jl, jr)
        # print("Desired relative pose: ")
        # print(desired_relative_pose)

        success_count = 0
        for _ in range(trial):
            # With noise
            jl_noise = get_noisy_solution(jl, sig_val=sigma_value)
            jr_noise = get_noisy_solution(jr)
            actual_relative_pose = rel_hole_wrt_peg(jl_noise, jr_noise)
            # print("Actual relative pose:")
            # print(actual_relative_pose)

            # Compute error
            err = norm(desired_relative_pose[:3, 3] - actual_relative_pose[:3, 3])
            # print(err)
            if err < tol:
                success_count += 1
        success_rate = success_count * 100 / trial
        print("Success rate: %2.6f" % success_rate)
        success_rate_list.append(success_rate)
    return success_rate_list


def compute_best_worst_success_rates():
    # Error tolerance
    tol_list = [0.0045, 0.005, 0.0055, 0.006, 0.0065, 0.007, 0.0075, 0.008, 0.0085, 0.009]

    # Compute success rate
    # Best IK pair
    js_left, js_right = left_arm_sols[best_indices[0]], right_arm_sols[best_indices[1]]
    print("Computing success rates for best solution")
    success_rate_list_best = compute_success_rates(js_left, js_right, tol_list, trial=10000)

    # Worst IK pair
    js_left, js_right = left_arm_sols[worst_indices[0]], right_arm_sols[worst_indices[1]]
    print("Computing success rates for worst solution")
    success_rate_list_worst = compute_success_rates(js_left, js_right, tol_list, trial=10000)

    # Plot
    plt.figure()
    plt.plot(tol_list, success_rate_list_best, "-bo", label=r"$\theta^*$")
    plt.plot(tol_list, success_rate_list_worst, "-ro", label=r"$\theta^-$")
    plt.title("Success rates with varying clearance between peg and hole")
    plt.legend(loc='upper left')
    plt.xlabel("clearance [m]")
    plt.ylabel("Percentage of success rates")
    plt.show()


# Compute success rate with varying Sigma
def compute_success_rate_varying_sigma():
    # Error tolerance list
    tol_list = [0.0045, 0.005, 0.0055, 0.006, 0.0065, 0.007, 0.0075, 0.008, 0.0085, 0.009]

    # Sigma list
    sigma_list = [0.0025, 0.0035, 0.0045, 0.0055, 0.0065, 0.0075, 0.0085, 0.0095]

    # Plot
    plt.figure()

    # Compute error with varying sigma
    success_save, count = {}, 0
    for s_val in sigma_list:
        # Compute success rate for both left and right arms
        # Best IK pair
        js_left, js_right = left_arm_sols[best_indices[0]], right_arm_sols[best_indices[1]]
        print("Computing success rate for sigma=%2.4f" % s_val)
        success_rate_list = compute_success_rates(js_left, js_right, tol_list, sigma_value=s_val, trial=1000)
        success_save[count] = success_rate_list
        count += 1
        plt.plot(tol_list, success_rate_list, "-o", label=r"$\sigma$="+str(s_val))
    plt.title("Success rates")
    plt.legend(ncol=5, loc="upper left")
    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left', ncol=4, mode="expand", borderaxespad=0.)
    plt.xlabel("clearance [m]")
    plt.ylabel("Percentage of success rates")
    plt.show()
    return success_save


# Compute success rates for best and worst solutions
compute_best_worst_success_rates()

# start_time = time.time()
# # Compute success rates for varying std of error
# success_rate_data = compute_success_rate_varying_sigma()
# dbfile2 = open("success_rate_varying_std", "ab")
# pickle.dump(success_rate_data, dbfile2)
# dbfile.close()
# end_time = time.time()
# print("Time elapsed: %2.6f" % (end_time - start_time))
