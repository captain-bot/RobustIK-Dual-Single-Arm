import robot_parameters
import numpy as np


# Function to check collision between square peg and hole
def collision_square(P_pegfinal, R_pegfinal, hole_width=robot_parameters.hole_width):
    # Compute vertices of peg-object in hole frame
    peg_pt = np.array([[-robot_parameters.peg_width / 2, robot_parameters.peg_width / 2],
                       [-robot_parameters.peg_width / 2, -robot_parameters.peg_width / 2],
                       [robot_parameters.peg_width / 2, -robot_parameters.peg_width / 2],
                       [robot_parameters.peg_width / 2, robot_parameters.peg_width / 2]]).T
    P_peg2d = P_pegfinal[:2].reshape(2, 1)
    R_peg2d = R_pegfinal[:2, :2]
    peg_pt = np.dot(R_peg2d, peg_pt) + P_peg2d

    # Compute vertices of hole in hole frame
    hole_pt = np.array([[-hole_width / 2, hole_width / 2],
                        [-hole_width / 2, -hole_width / 2],
                        [hole_width / 2, -hole_width / 2],
                        [hole_width / 2, hole_width / 2]]).T

    # Check each peg_pt lies inside the region defined by hole_pt
    max_x, max_y = np.max(hole_pt[0, :]), np.max(hole_pt[1, :])
    min_x, min_y = np.min(hole_pt[0, :]), np.min(hole_pt[1, :])
    # print("max_x: %2.4f, max_y: %2.4f" % (max_x, max_y))
    # print("min_x: %2.4f, min_y: %2.4f" % (min_x, min_y))

    collision = False
    for i in range(peg_pt.shape[1]):
        if peg_pt[0, i] > max_x or peg_pt[0, i] < min_x:
            collision = True

        if peg_pt[1, i] > max_y or peg_pt[1, i] < min_y:
            collision = True
    return collision
