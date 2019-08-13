import numpy as np

"""
This file consists of the following parameters
1. peg length and hole length for manipulation
2. robot noise parameters used in simulation
3. joint axis and position of Baxter arm (note: these parameter values are same if they are taken wrt their respective 
arm mount frames)
4. "T_base2leftarmmount" and "T_base2rightarmmount" values
"""

# Length of hole and peg
l_peg, l_hole = 0.100, 0.100

# Radius of Peg and Hole
rad_peg, rad_hole = 0.020, 0.0270

# Joint errors
mu, sig, num_std = 0, 0.0035, 3
c_val = (num_std * sig)**2

# Define axis of rotations of each joint (wrt arm_mount frame)
w1r, w2r, w3r = [0, 0, 1], [0, 1, 0], [1, 0, 0]
w4r = w6r = w2r
w5r = w7r = w3r
wr = np.array([w1r, w2r, w3r, w4r, w5r, w6r, w7r])

# Define location of origins of each joints (wrt arm_mount frame)
q1r = [0.056, 0.000, 0.011]
q2r = [0.125, 0.000, 0.281]
q3r = [0.227, 0.000, 0.281]
q4r = [0.489, 0.000, 0.213]
q5r = [0.593, -0.001, 0.213]
q6r = [0.863, -0.001, 0.195]
q7r = [0.979, -0.002, 0.195]
qr = np.array([q1r, q2r, q3r, q4r, q5r, q6r, q7r])

# Define g_st0 (gripper frame wrt arm_mount at zero joint angle)
# Note: this g_st0 is valid for standard_narrow gripper and both the arms
# have same g_st0 wrt arm_mount frame
gst0 = np.array([[0, 0, 1, 1.213],
        [0.0, 1.0, 0.0, -0.002],
        [-1.0, 0.0, 0.0, 0.190],
        [0.0, 0.0, 0.0, 1.0]])

# Transformations from arm_mount to base frame for left and right arm
T_base2leftarmmount = np.array([[0.707, -0.707, 0, 0.025],
                                [0.707, 0.707, 0, 0.220],
                                [0, 0, 1, 0.109],
                                [0, 0, 0, 1]])

T_base2rightarmmount = np.array([[0.707, 0.707, 0, 0.025],
                                [-0.707, 0.707, 0, -0.220],
                                [0, 0, 1, 0.109],
                                [0, 0, 0, 1]])

# Transform joint axes and origin locations into base frame for both arms
wr_l = np.dot(T_base2leftarmmount[:3, :3], wr.T).T
qr_l = (np.dot(T_base2leftarmmount[:3, :3], qr.T) + T_base2leftarmmount[:3, 3].reshape(3, 1)).T
gst0_l = np.dot(T_base2leftarmmount, gst0)

wr_r = np.dot(T_base2rightarmmount[:3, :3], wr.T).T
qr_r = (np.dot(T_base2rightarmmount[:3, :3], qr.T) + T_base2rightarmmount[:3, 3].reshape(3, 1)).T
gst0_r = np.dot(T_base2rightarmmount, gst0)
