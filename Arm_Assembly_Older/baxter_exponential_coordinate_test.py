from kinematic_utilities import *

th_ang_left = [0.1, -0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
# th_ang_left = [0]*7

# Robot parameters
trans_base2armmount = \
    np.array([[0.7071, -0.7071, 0.0, 0.0254],
            [0.7071, 0.7071, 0.0, 0.2190],
            [0.0, 0.0, 1.0, 0.108],
            [0, 0, 0, 1.0]])

# Define g_st0 (gripper frame wrt arm_mount at zero joint angle)
# Note: this g_st0 is valid for standard_narrow gripper
g_st0_left = np.array([[0, 0, 1, 1.213],
                [0.0, 1.0, 0.0, -0.002],
                [-1.0, 0.0, 0.0, 0.190],
                [0.0, 0.0, 0.0, 1.0]])

# Define axis of rotations of each joint
w1r, w2r, w3r = [0, 0, 1], [0, 1, 0], [1, 0, 0]
w4r = w6r = w2r
w5r = w7r = w3r
wr = np.array([w1r, w2r, w3r, w4r, w5r, w6r, w7r])

# Define location of origins of each joints
q1r = [0.056, 0.000, 0.011]
q2r = [0.125, 0.000, 0.281]
q3r = [0.227, 0.000, 0.281]
q4r = [0.489, 0.000, 0.213]
q5r = [0.593, -0.001, 0.213]
q6r = [0.863, -0.001, 0.195]
q7r = [0.979, -0.002, 0.195]
qr = np.array([q1r, q2r, q3r, q4r, q5r, q6r, q7r])

# Solve direct position kinematics
gst_gripper2armmount, transform_upto_joint_left = exp_direct_kin(g_st0_left, wr, qr, th_ang_left)
print("Gripper pose wrt arm-mount-frame: ")
print(gst_gripper2armmount)
# print("Transform upto")
# print(transform_upto_joint_left)

# All transformations that we got above this line are wrt arm_mount frame
# Det end-effector transform wrt base frame
gst_gripper2base = np.dot(trans_base2armmount, gst_gripper2armmount)
print("Gripper pose wrt base frame: ")
print(gst_gripper2base)

# Compute spatial-jacobian of Baxter robot
spatial_jac = velocity_direct_kin(g_st0_left, wr, qr, th_ang_left)
print("Spatial Jacobian wrt arm-mount-frame: ")
print(spatial_jac)
