import numpy as np
from numpy.linalg import inv
import math
import ikModuleLeft as ikfl
import ikModuleRight as ikfr


"""
This file contains several important functions which can be used as a stand along function 
or in conjunction with other functions. Below we list name of all the available functions 
along with small descriptions of their utilities,

1. exp_twist(xi, theta): Finds exponential of twist vector
2. exp_direct_kin(gst0, joint_axes, q_axes, th): Finds forward kinematics using Exponential coordinates
3. velocity_direct_kin(gst0, joint_axes, q_axes, th): Finds spatial Jacobian using Exponential coordinates
4. unitquat2rotm(q): Quaternion to rotation matrix
5. rotm2quat(rotm): Rotation matrix to quaternion
6. get_adjoint(g): Compute adjoint matrix given a transformation
7. relative_jac(gst0_l, gst0_r, gst_l, gst_r, transform_upto_l, transform_upto_r, joint_axes, q_axes, theta): Compute relative jacobian
8. skew_quat(q): Compute skew-symmetric quaternion
9. norm(v): Compute norm of a vector
10. position_err_bound(jp, cval): Position error bound
11. rotation_err_bound(jr, cval, qd): Orientation error bound
12. get_noisy_solution_both(sol_l, sol_r, sig_val=0.0035): given pure IK-pair returns noisy IK-pair
"""


# Finds exponential of twist vector
def exp_twist(xi, theta):
    omega = xi[3:]
    v = xi[:3]

    omega_hat = np.array([[0, -omega[2], omega[1]],
                          [omega[2], 0, -omega[0]],
                          [-omega[1], omega[0], 0]])

    omega_hat_theta = np.identity(3) + np.sin(theta)*omega_hat + (1-np.cos(theta))*np.dot(omega_hat, omega_hat)
    p_val = np.dot((np.identity(3) - omega_hat_theta), np.cross(omega, v)) + np.dot(np.outer(omega, omega), v)*theta
    gab = np.vstack((np.hstack((omega_hat_theta, p_val.reshape(3, 1))), np.array([[0, 0, 0, 1]])))
    return gab


# Finds forward kinematics using Exponential coordinates
def exp_direct_kin(gst0, joint_axes, q_axes, th):
    dim = 3
    num_of_joints = len(th)
    gst_temp = np.identity(dim+1)
    tran_upto_joint = np.zeros((num_of_joints+1, dim+1, dim+1))

    for i in range(num_of_joints):
        tran_upto_joint[i, :, :] = gst_temp
        omega = joint_axes[i, :]
        q = q_axes[i, :]
        xi = np.hstack((np.cross(-omega, q), omega))
        gst_joint_i = exp_twist(xi, th[i])
        gst_temp = np.dot(gst_temp, gst_joint_i)

    tran_upto_joint[num_of_joints, :, :] = gst_temp
    gst = np.dot(gst_temp, gst0)
    return gst, tran_upto_joint


# Finds spatial Jacobian using Exponential coordinates
def velocity_direct_kin(gst0, joint_axes, q_axes, th):
    dim = 3
    num_of_joints = len(th)
    spatial_jac = np.zeros((dim+3, num_of_joints))

    gst, transform_upto_joint = exp_direct_kin(gst0, joint_axes, q_axes, th)
    for i in range(num_of_joints):
        if i > 0:
            g = transform_upto_joint[i, :, :]
            R = g[:3, :3]
            p = g[:3, 3]
            p_hat = np.array([[0, -p[2], p[1]],
                              [p[2], 0, -p[0]],
                              [-p[1], p[0], 0]])
            temp1 = np.dot(p_hat, R)
            Ad_g = np.hstack((R, temp1))
            temp2 = np.hstack((np.zeros((3, 3)), R))
            Ad_g = np.vstack((Ad_g, temp2))

        omega = joint_axes[i, :]
        q = q_axes[i, :]
        xi = np.hstack((np.cross(-omega, q), omega))

        if i > 0:
            xi_prime = np.dot(Ad_g, xi)
        else:
            xi_prime = xi

        spatial_jac[:, i] = xi_prime

    # Compute analytical jacobian
    p = gst[:3, 3]
    p_hat = np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])
    pre_mult_mat = np.vstack((np.hstack((np.eye(3), -p_hat)), np.hstack((np.zeros((3, 3)), np.eye(3)))))
    analytical_jac = np.dot(pre_mult_mat, spatial_jac)

    return spatial_jac, analytical_jac


# Quaternion to rotation matrix
def unitquat2rotm(q):
    return np.array([[math.pow(q[0], 2) + math.pow(q[1], 2) - math.pow(q[2], 2) - math.pow(q[3], 2),
            2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[1] * q[3] + 2 * q[0] * q[2]],
            [2 * q[1] * q[2] + 2 * q[0] * q[3],
            math.pow(q[0], 2) - math.pow(q[1], 2) + math.pow(q[2], 2) - math.pow(q[3], 2),
            2 * q[2] * q[3] - 2 * q[0] * q[1]],
            [2 * q[1] * q[3] - 2 * q[0] * q[2], 2 * q[2] * q[3] + 2 * q[0] * q[1],
            math.pow(q[0], 2) - math.pow(q[1], 2) - math.pow(q[2], 2) + math.pow(q[3], 2)]])


# Rotation matrix to quaternion
def rotm2quat(rotm):
    if (rotm[0, 0] + rotm[1, 1] + rotm[2, 2] - 1) / 2 < -1 or (rotm[0, 0] + rotm[1, 1] + rotm[2, 2] - 1) / 2 > 1:
        return None
    angle = math.acos((rotm[0, 0] + rotm[1, 1] + rotm[2, 2] - 1) / 2)
    # print(angle)
    temp = math.sqrt((rotm[2, 1] - rotm[1, 2])**2 + (rotm[0, 2] - rotm[2, 0])**2 + (rotm[1, 0] - rotm[0, 1])**2)
    wx = (rotm[2, 1] - rotm[1, 2]) / temp
    wy = (rotm[0, 2] - rotm[2, 0]) / temp
    wz = (rotm[1, 0] - rotm[0, 1]) / temp
    quat = np.array([math.cos(angle / 2), wx * math.sin(angle / 2), wy * math.sin(angle / 2), wz * math.sin(angle / 2)])
    return quat


# Compute adjoint matrix given a transformation
def get_adjoint(g):
    R, p = g[:3, :3], g[:3, 3]
    p_hat = np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])
    temp1 = np.dot(p_hat, R)
    temp1 = np.hstack((R, temp1))
    temp2 = np.hstack((np.zeros((3, 3)), R))
    return np.vstack((temp1, temp2))


# Compute relative jacobian
def relative_jac(gst0_l, gst0_r, gst_l, gst_r, transform_upto_l, transform_upto_r, joint_axes, q_axes, theta):
    dim = 3
    num_of_joints = len(theta)
    spatial_jac = np.zeros((dim + 3, num_of_joints))

    # Fill-in the first 7 columns of spatial jacobian matrix
    num = num_of_joints//2
    for i in range(num):
        if i < num - 1:
            g = np.dot(inv(transform_upto_l[i+1, :, :]), gst_l)
        else:
            g = gst0_l
        Ad_g = get_adjoint(g)
        Ad_g_inv = np.dot(inv(Ad_g), np.eye(6))

        omega = joint_axes[i, :]
        q = q_axes[i, :]
        xi = np.hstack((np.cross(-omega, q), omega))

        xi_prime = -np.dot(Ad_g_inv, xi)
        spatial_jac[:, num - 1 - i] = xi_prime

    # Fill-in the last 7 columns of spatial jacobian matrix
    for j in range(num):
        if j > 0:
            g = np.dot(inv(gst_l), transform_upto_r[j+1, :, :])
        else:
            g = gst_l
        Ad_g = get_adjoint(g)
        omega = joint_axes[j + num, :]
        q = q_axes[j + num, :]
        xi = np.hstack((np.cross(-omega, q), omega))

        if j > 0:
            xi_prime = np.dot(Ad_g, xi)
        else:
            Ad_g_inv = np.dot(inv(Ad_g), np.eye(6))
            xi_prime = np.dot(Ad_g_inv, xi)

        spatial_jac[:, num + j] = xi_prime

    # Compute analytical jacobian and return
    g_rel = np.dot(inv(gst_l), gst_r)
    p = g_rel[:3, 3]
    p_hat = np.array([[0, -p[2], p[1]], [p[2], 0, -p[0]], [-p[1], p[0], 0]])
    pre_mult_mat = np.vstack((np.hstack((np.eye(3), -p_hat)), np.hstack((np.zeros((3, 3)), np.eye(3)))))
    analytical_jac = np.dot(pre_mult_mat, spatial_jac)
    return analytical_jac, spatial_jac


# Compute skew-symmetric quaternion
def skew_quat(q):
    return np.array([[-q[1], q[0], -q[3], q[2]],
                     [-q[2], q[3], q[0], -q[1]],
                     [-q[3], -q[2], q[1], q[0]]])
    # return np.array([[-q[1], q[0], q[3], -q[2]],
    #                 [-q[2], -q[3], q[0], q[1]],
    #                 [-q[3], q[2], -q[1], q[0]]])


# Compute norm of a vector
def norm(v):
    return math.sqrt(np.dot(v, v))


# Position error bound
def position_err_bound(jp, cval):
    w, v = np.linalg.eig(np.dot(jp, jp.T))
    max_id = np.argmax(w)
    max_eig = w[max_id]
    return math.sqrt(cval*max_eig)


# Orientation error bound
def rotation_err_bound(jr, cval, qd):
    Hd = skew_quat(qd)
    w, v = np.linalg.eig(np.dot(jr, jr.T))
    max_id = np.argmax(w)
    max_eig = w[max_id]
    max_v = v[:, max_id]
    v_vec = (1/2)*math.sqrt(cval*max_eig)*max_v
    q_star = qd + np.dot(v_vec.T, Hd)
    q_star = q_star/norm(q_star)
    qd /= norm(qd)
    worst_rot_err = np.arccos(np.dot(q_star, qd.T))
    return worst_rot_err, q_star


# Compute Inverse-Kinematics solution using IK-Fast
# Note: pd and qd are desired poses of gripper frame wrt arm_mount frame
def get_iksol(pd, qd, adjust_len=0.067, side="left", config_num=1):
    unit_q = qd/norm(qd)
    R_mat = unitquat2rotm(unit_q)
    P_ikfast = pd - adjust_len * R_mat[:, 2]
    input_pose_matrix = np.hstack((R_mat, P_ikfast.reshape(3, 1)))
    ikfast_pose_flattened = input_pose_matrix.reshape(12, ).tolist()
    if side != "left":
        config_num = 2
        ikfr.compIKs(config_num, ikfast_pose_flattened)
    else:
        ikfl.compIKs(config_num, ikfast_pose_flattened)
    print("Solutions written in ik_sol_config%d.txt" % config_num)


# Function to get noisy solution from a pure solution
def get_noisy_solution_both(sol_l, sol_r, sig_val=0.0035):
    noise_l = np.random.normal(0, sig_val, len(sol_l))
    noise_r = np.random.normal(0, sig_val, len(sol_r))
    sol_noisy_l, sol_noisy_r = [], []
    for indx in range(7):
        sol_noisy_l.append(sol_l[indx] + noise_l[indx])
        sol_noisy_r.append(sol_r[indx] + noise_r[indx])
    return sol_noisy_l, sol_noisy_r
