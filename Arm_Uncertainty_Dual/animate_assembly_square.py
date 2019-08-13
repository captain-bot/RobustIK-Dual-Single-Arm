import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
# import kinematic_utilities
import robot_parameters
import matplotlib.patches as patches
import matplotlib as mpl

# Number of points to generate grid
npt = 3


# Function to see animation for square-peg in square-hole
def view_animation_square(T_leftgripper, T_rightgripper, T_peg, T_hole, dist_peg_hole):
    lin_vel = 0.002
    num_iteration = dist_peg_hole // lin_vel

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ################################################################
    ########### Plot Hole Cross-section with Frames ################
    ################################################################
    # Face plate
    pt_x = np.linspace(-robot_parameters.hole_width/2, robot_parameters.hole_width/2, npt)
    pt_y = np.linspace(-robot_parameters.hole_width/2, robot_parameters.hole_width/2, npt)
    x_grid, y_grid = np.meshgrid(pt_x, pt_y)
    z_grid = robot_parameters.l_hole * np.ones(x_grid.shape)
    pts = np.vstack((x_grid.reshape(1, x_grid.size), y_grid.reshape(1, y_grid.size), z_grid.reshape(1, z_grid.size)))
    pts = np.dot(T_rightgripper[:3, :3], pts) + T_rightgripper[:3, 3].reshape(3, 1)
    x_grid = pts[0, :].reshape(npt, npt)
    y_grid = pts[1, :].reshape(npt, npt)
    z_grid = pts[2, :].reshape(npt, npt)

    # Surface-1
    pt_x1 = np.linspace(-robot_parameters.hole_width/2, robot_parameters.hole_width/2, npt)
    pt_z1 = np.linspace(0, robot_parameters.l_hole, npt)
    x1_grid, z1_grid = np.meshgrid(pt_x1, pt_z1)
    y1_grid = (robot_parameters.hole_width/2) * np.ones(x1_grid.shape)
    pts1 = np.vstack((x1_grid.reshape(1, x1_grid.size), y1_grid.reshape(1, y1_grid.size), z1_grid.reshape(1, z1_grid.size)))
    pts1 = np.dot(T_rightgripper[:3, :3], pts1) + T_rightgripper[:3, 3].reshape(3, 1)
    x1_grid = pts1[0, :].reshape(npt, npt)
    y1_grid = pts1[1, :].reshape(npt, npt)
    z1_grid = pts1[2, :].reshape(npt, npt)

    # Surface-2
    pt_x2 = np.linspace(-robot_parameters.hole_width / 2, robot_parameters.hole_width / 2, npt)
    pt_z2 = np.linspace(0, robot_parameters.l_hole, npt)
    x2_grid, z2_grid = np.meshgrid(pt_x2, pt_z2)
    y2_grid = -(robot_parameters.hole_width / 2) * np.ones(x2_grid.shape)
    pts2 = np.vstack(
        (x2_grid.reshape(1, x2_grid.size), y2_grid.reshape(1, y2_grid.size), z2_grid.reshape(1, z2_grid.size)))
    pts2 = np.dot(T_rightgripper[:3, :3], pts2) + T_rightgripper[:3, 3].reshape(3, 1)
    x2_grid = pts2[0, :].reshape(npt, npt)
    y2_grid = pts2[1, :].reshape(npt, npt)
    z2_grid = pts2[2, :].reshape(npt, npt)

    # Surface-3
    pt_y3 = np.linspace(-robot_parameters.hole_width / 2, robot_parameters.hole_width / 2, npt)
    pt_z3 = np.linspace(0, robot_parameters.l_hole, npt)
    y3_grid, z3_grid = np.meshgrid(pt_y3, pt_z3)
    x3_grid = (robot_parameters.hole_width / 2) * np.ones(y3_grid.shape)
    pts3 = np.vstack(
        (x3_grid.reshape(1, x3_grid.size), y3_grid.reshape(1, y3_grid.size), z3_grid.reshape(1, z3_grid.size)))
    pts3 = np.dot(T_rightgripper[:3, :3], pts3) + T_rightgripper[:3, 3].reshape(3, 1)
    x3_grid = pts3[0, :].reshape(npt, npt)
    y3_grid = pts3[1, :].reshape(npt, npt)
    z3_grid = pts3[2, :].reshape(npt, npt)

    # Surface-4
    pt_y4 = np.linspace(-robot_parameters.hole_width / 2, robot_parameters.hole_width / 2, npt)
    pt_z4 = np.linspace(0, robot_parameters.l_hole, npt)
    y4_grid, z4_grid = np.meshgrid(pt_y4, pt_z4)
    x4_grid = -(robot_parameters.hole_width / 2) * np.ones(y4_grid.shape)
    pts4 = np.vstack(
        (x4_grid.reshape(1, x4_grid.size), y4_grid.reshape(1, y4_grid.size), z4_grid.reshape(1, z4_grid.size)))
    pts4 = np.dot(T_rightgripper[:3, :3], pts4) + T_rightgripper[:3, 3].reshape(3, 1)
    x4_grid = pts4[0, :].reshape(npt, npt)
    y4_grid = pts4[1, :].reshape(npt, npt)
    z4_grid = pts4[2, :].reshape(npt, npt)

    # Plot Hole-object
    ax.plot_surface(x_grid, y_grid, z_grid, alpha=0.5, color='w')
    ax.plot_surface(x1_grid, y1_grid, z1_grid, alpha=0.5, color='b')
    ax.plot_surface(x2_grid, y2_grid, z2_grid, alpha=0.5, color='b')
    ax.plot_surface(x3_grid, y3_grid, z3_grid, alpha=0.5, color='b')
    ax.plot_surface(x4_grid, y4_grid, z4_grid, alpha=0.5, color='b')

    # Plot hole tip frame
    ax.quiver(T_hole[0, 3], T_hole[1, 3], T_hole[2, 3], T_hole[0, 0], T_hole[1, 0],
              T_hole[2, 0], length=0.01, normalize=False, color='r')
    ax.quiver(T_hole[0, 3], T_hole[1, 3], T_hole[2, 3], T_hole[0, 1], T_hole[1, 1],
              T_hole[2, 1], length=0.01, normalize=False, color='g')
    ax.quiver(T_hole[0, 3], T_hole[1, 3], T_hole[2, 3], T_hole[0, 2], T_hole[1, 2],
              T_hole[2, 2], length=0.01, normalize=False, color='b')

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title("Square Peg into Square Hole Assembly Simulation")

    ax.set_xlim(0.65, 0.85)
    ax.set_ylim(-0.01, 0.18)
    ax.set_zlim(0.4, 0.5)
    ax.view_init(56, 141)

    ################################################################
    ########### Plot Peg Cross-section with Frames #################
    ################################################################
    itr = 0
    while itr <= num_iteration:
        # Peg face plate
        pt_x = np.linspace(-robot_parameters.peg_width/2, robot_parameters.peg_width/2, npt)
        pt_y = np.linspace(-robot_parameters.peg_width/2, robot_parameters.peg_width/2, npt)
        x_grid, y_grid = np.meshgrid(pt_x, pt_y)
        z_grid = robot_parameters.l_peg*np.ones(x_grid.shape)

        pts = np.vstack((x_grid.reshape(1, x_grid.size), y_grid.reshape(1, y_grid.size), z_grid.reshape(1, z_grid.size)))
        pts = np.dot(T_leftgripper[:3, :3], pts) + T_leftgripper[:3, 3].reshape(3, 1)

        x_grid = pts[0, :].reshape(npt, npt)
        y_grid = pts[1, :].reshape(npt, npt)
        z_grid = pts[2, :].reshape(npt, npt)

        # Surface-1
        pt_x1p = np.linspace(-robot_parameters.peg_width / 2, robot_parameters.peg_width / 2, npt)
        pt_z1p = np.linspace(0, robot_parameters.l_peg, npt)
        x1p_grid, z1p_grid = np.meshgrid(pt_x1p, pt_z1p)
        y1p_grid = (robot_parameters.peg_width / 2) * np.ones(x1p_grid.shape)
        pts1p = np.vstack(
            (x1p_grid.reshape(1, x1p_grid.size), y1p_grid.reshape(1, y1p_grid.size), z1p_grid.reshape(1, z1p_grid.size)))
        pts1p = np.dot(T_leftgripper[:3, :3], pts1p) + T_leftgripper[:3, 3].reshape(3, 1)
        x1p_grid = pts1p[0, :].reshape(npt, npt)
        y1p_grid = pts1p[1, :].reshape(npt, npt)
        z1p_grid = pts1p[2, :].reshape(npt, npt)

        # Surface-2
        pt_x2p = np.linspace(-robot_parameters.peg_width / 2, robot_parameters.peg_width / 2, npt)
        pt_z2p = np.linspace(0, robot_parameters.l_peg, npt)
        x2p_grid, z2p_grid = np.meshgrid(pt_x2p, pt_z2p)
        y2p_grid = -(robot_parameters.peg_width / 2) * np.ones(x2p_grid.shape)
        pts2p = np.vstack(
            (x2p_grid.reshape(1, x2p_grid.size), y2p_grid.reshape(1, y2p_grid.size), z2p_grid.reshape(1, z2p_grid.size)))
        pts2p = np.dot(T_leftgripper[:3, :3], pts2p) + T_leftgripper[:3, 3].reshape(3, 1)
        x2p_grid = pts2p[0, :].reshape(npt, npt)
        y2p_grid = pts2p[1, :].reshape(npt, npt)
        z2p_grid = pts2p[2, :].reshape(npt, npt)

        # Surface-3
        pt_y3p = np.linspace(-robot_parameters.peg_width / 2, robot_parameters.peg_width / 2, npt)
        pt_z3p = np.linspace(0, robot_parameters.l_peg, npt)
        y3p_grid, z3p_grid = np.meshgrid(pt_y3p, pt_z3p)
        x3p_grid = (robot_parameters.peg_width / 2) * np.ones(y3p_grid.shape)
        pts3p = np.vstack(
            (x3p_grid.reshape(1, x3p_grid.size), y3p_grid.reshape(1, y3p_grid.size), z3p_grid.reshape(1, z3p_grid.size)))
        pts3p = np.dot(T_leftgripper[:3, :3], pts3p) + T_leftgripper[:3, 3].reshape(3, 1)
        x3p_grid = pts3p[0, :].reshape(npt, npt)
        y3p_grid = pts3p[1, :].reshape(npt, npt)
        z3p_grid = pts3p[2, :].reshape(npt, npt)

        # Surface-4
        pt_y4p = np.linspace(-robot_parameters.peg_width / 2, robot_parameters.peg_width / 2, npt)
        pt_z4p = np.linspace(0, robot_parameters.l_peg, npt)
        y4p_grid, z4p_grid = np.meshgrid(pt_y4p, pt_z4p)
        x4p_grid = -(robot_parameters.peg_width / 2) * np.ones(y4p_grid.shape)
        pts4p = np.vstack(
            (x4p_grid.reshape(1, x4p_grid.size), y4p_grid.reshape(1, y4p_grid.size), z4p_grid.reshape(1, z4p_grid.size)))
        pts4p = np.dot(T_leftgripper[:3, :3], pts4p) + T_leftgripper[:3, 3].reshape(3, 1)
        x4p_grid = pts4p[0, :].reshape(npt, npt)
        y4p_grid = pts4p[1, :].reshape(npt, npt)
        z4p_grid = pts4p[2, :].reshape(npt, npt)

        # Plot Peg-object
        peg_face = ax.plot_surface(x_grid, y_grid, z_grid, alpha=0.5, color='w')
        peg_surface1 = ax.plot_surface(x1p_grid, y1p_grid, z1p_grid, alpha=0.5, color='r')
        peg_surface2 = ax.plot_surface(x2p_grid, y2p_grid, z2p_grid, alpha=0.5, color='r')
        peg_surface3 = ax.plot_surface(x3p_grid, y3p_grid, z3p_grid, alpha=0.5, color='r')
        peg_surface4 = ax.plot_surface(x4p_grid, y4p_grid, z4p_grid, alpha=0.5, color='r')

        # Plot peg tip frame
        peg_x = ax.quiver(T_peg[0, 3], T_peg[1, 3], T_peg[2, 3], T_peg[0, 0], T_peg[1, 0],
                  T_peg[2, 0], length=0.01, normalize=False, color='r')
        peg_y = ax.quiver(T_peg[0, 3], T_peg[1, 3], T_peg[2, 3], T_peg[0, 1], T_peg[1, 1],
                  T_peg[2, 1], length=0.01, normalize=False, color='g')
        peg_z = ax.quiver(T_peg[0, 3], T_peg[1, 3], T_peg[2, 3], T_peg[0, 2], T_peg[1, 2],
                  T_peg[2, 2], length=0.01, normalize=False, color='b')

        # Move left gripper frame
        T_leftgripper[:3, 3] += lin_vel*T_leftgripper[:3, 2]
        T_peg[:3, 3] += lin_vel * T_peg[:3, 2]

        # Hold the plot for 0.005s
        plt.pause(0.001)

        if not itr > num_iteration:
            peg_face.remove()
            peg_surface1.remove()
            peg_surface2.remove()
            peg_surface3.remove()
            peg_surface4.remove()
            peg_x.remove()
            peg_y.remove()
            peg_z.remove()

        itr += 1


def visualize_all(T_base2leftgripper, T_base2rightgripper, T_base2peg, T_base2hole, dist, P_pegfinal, R_pegfinal):
    view_animation_square(T_base2leftgripper, T_base2rightgripper, T_base2peg, T_base2hole, dist)

    # Compute vertices of peg-object in hole frame
    peg_pt = np.array([[-robot_parameters.peg_width / 2, robot_parameters.peg_width / 2],
                       [-robot_parameters.peg_width / 2, -robot_parameters.peg_width / 2],
                       [robot_parameters.peg_width / 2, -robot_parameters.peg_width / 2],
                       [robot_parameters.peg_width / 2, robot_parameters.peg_width / 2]]).T
    P_peg2d = P_pegfinal[:2].reshape(2, 1)
    R_peg2d = R_pegfinal[:2, :2]
    peg_pt = np.dot(R_peg2d, peg_pt) + P_peg2d
    # print(peg_pt)

    # Compute vertices of hole in hole frame
    hole_pt = np.array([[-robot_parameters.hole_width / 2, robot_parameters.hole_width / 2],
                        [-robot_parameters.hole_width / 2, -robot_parameters.hole_width / 2],
                        [robot_parameters.hole_width / 2, -robot_parameters.hole_width / 2],
                        [robot_parameters.hole_width / 2, robot_parameters.hole_width / 2]]).T

    # # Check each peg_pt lies inside the region defined by hole_pt
    max_x, max_y = np.max(hole_pt[0, :]), np.max(hole_pt[1, :])
    min_x, min_y = np.min(hole_pt[0, :]), np.min(hole_pt[1, :])

    # Visualize
    fig, ax = plt.subplots(1)

    # print("hole_pt[1, 0]: %2.4f" % hole_pt[1, 0])
    rect_hole = patches.Rectangle((hole_pt[0, 1], hole_pt[1, 1]), robot_parameters.hole_width,
                                  robot_parameters.hole_width, linewidth=1, alpha=0.3, edgecolor='b', facecolor='b',
                                  label="hole c/s")
    rect_peg = patches.Rectangle(((-robot_parameters.peg_width / 2) + P_peg2d[0, 0],
                                  (-robot_parameters.peg_width / 2) + P_peg2d[1, 0]), robot_parameters.peg_width,
                                 robot_parameters.peg_width, linewidth=1, alpha=0.3, edgecolor='r', facecolor='r',
                                 label="peg c/s")

    th = np.arctan2(R_peg2d[1, 0], R_peg2d[0, 0])
    th_d = th * 180 / np.pi
    
    # print("Peg orientation error: %2.4f deg" % (th_d))

    t2 = mpl.transforms.Affine2D().rotate_deg(th_d) + ax.transData
    rect_peg.set_transform(t2)

    ax.add_patch(rect_hole)
    ax.add_patch(rect_peg)

    ax.set_xlabel("hole_x [m]")
    ax.set_ylabel("hole_y [m]")
    ax.set_xlim(min_x - 0.010, max_x + 0.010)
    ax.set_ylim(min_y - 0.010, max_y + 0.010)
    ax.set_aspect('equal')
    ax.set_title("Peg and hole cross-section view")
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.0)
    plt.show()
