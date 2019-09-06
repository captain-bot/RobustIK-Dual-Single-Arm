import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import robot_parameters
from matplotlib import cm
import kinematic_utilities

"""
This file contains the relevant functions to visualize peg in hole assembly process with cylindrical peg and hole.
"""


def data_for_cylinder_along_z(center_x, center_y, radius, height_z):
    z = np.linspace(0, height_z, 20)
    theta = np.linspace(0, 2*np.pi, 20)
    theta_grid, z_grid = np.meshgrid(theta, z)
    x_grid = radius*np.cos(theta_grid) + center_x
    y_grid = radius*np.sin(theta_grid) + center_y
    return x_grid, y_grid, z_grid


def generate_cylinder_pts(T, radius, length):
    rot_mat = T[:3, :3]
    Xc, Yc, Zc = data_for_cylinder_along_z(0.0, 0.0, radius, length)

    # Create points for rotated cylinder
    new_Xc = Xc.reshape((1, 20*20))
    new_Yc = Yc.reshape((1, 20*20))
    new_Zc = Zc.reshape((1, 20*20))
    all_new = np.vstack((new_Xc, new_Yc, new_Zc))
    all_rot = np.dot(rot_mat, all_new)

    rot_Xc = (all_rot[0, :] + T[0, 3]).reshape(20, 20)
    rot_Yc = (all_rot[1, :] + T[1, 3]).reshape(20, 20)
    rot_Zc = (all_rot[2, :] + T[2, 3]).reshape(20, 20)

    return rot_Xc, rot_Yc, rot_Zc


# Transformations of peg and hole wrt base frame
def view_animation(T_leftgripper, T_rightgripper, T_peg, T_hole, dist_peg_hole):
    # Settings (newly added for video)
    video_file = "myvid.mp4"
    clear_frames = True  # Should it clear the figure between each frame?
    fps = 5

    # Output video writer
    FFMpegWriter = animation.writers['ffmpeg']
    metadata = dict(title='Movie Test', artist='Matplotlib', comment='Movie support!')
    writer = FFMpegWriter(fps=fps, metadata=metadata)
    #########################################

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    lin_vel = 0.002
    num_iteration = dist_peg_hole // lin_vel

    with writer.saving(fig, video_file, 200):
        Xh, Yh, Zh = generate_cylinder_pts(T_rightgripper, robot_parameters.rad_hole, robot_parameters.l_hole)
        h2 = ax.plot_surface(Xh, Yh, Zh, alpha=0.5, color='b')
        # ax.set_xlim(0.0, 0.85)
        # ax.set_ylim(-0.20, -0.01)
        # ax.set_zlim(0.0, 0.45)
        ax.set_xlim(0.6, 0.9)
        ax.set_ylim(-0.01, 0.2)
        ax.set_zlim(0.3, 0.5)
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        ax.set_title('Peg-in-Hole Assembly Simulation')
        ax.view_init(30, 160)

        # Plot hole frame axes
        ax.quiver(T_hole[0, 3], T_hole[1, 3], T_hole[2, 3], T_hole[0, 0], T_hole[1, 0],
                  T_hole[2, 0], length=0.05, normalize=False, color='r')
        ax.quiver(T_hole[0, 3], T_hole[1, 3], T_hole[2, 3], T_hole[0, 1], T_hole[1, 1],
                  T_hole[2, 1], length=0.05, normalize=False, color='g')
        ax.quiver(T_hole[0, 3], T_hole[1, 3], T_hole[2, 3], T_hole[0, 2], T_hole[1, 2],
                  T_hole[2, 2], length=0.05, normalize=False, color='b')

        # Compute Euclidean distance between peg and hole
        print(T_hole[:3, 3])
        print(T_peg[:3, 3])
        peg_hole_euclidean_dist = kinematic_utilities.norm(T_hole[:3, 3] - T_peg[:3, 3])
        print(peg_hole_euclidean_dist)
        itr = 0
        while peg_hole_euclidean_dist > 1e-3 and itr <= num_iteration:
            Xp, Yp, Zp = generate_cylinder_pts(T_leftgripper, robot_parameters.rad_peg, robot_parameters.l_peg)
            h1 = ax.plot_surface(Xp, Yp, Zp, alpha=0.5, color='r')

            # Plot hole frame axes
            peg_x = ax.quiver(T_peg[0, 3], T_peg[1, 3], T_peg[2, 3], T_peg[0, 0], T_peg[1, 0],
                      T_peg[2, 0], length=0.05, normalize=False, color='r')
            peg_y = ax.quiver(T_peg[0, 3], T_peg[1, 3], T_peg[2, 3], T_peg[0, 1], T_peg[1, 1],
                      T_peg[2, 1], length=0.05, normalize=False, color='g')
            peg_z = ax.quiver(T_peg[0, 3], T_peg[1, 3], T_peg[2, 3], T_peg[0, 2], T_peg[1, 2],
                      T_peg[2, 2], length=0.05, normalize=False, color='b')

            # Plot number of iterations and current Euclidean distance
            itr_text = ax.text(0.75, 0.075, 0.4, "iteration: " + str(itr), color='red')
            # dist_str = "2.4f" % peg_hole_euclidean_dist
            dist_text = ax.text(0.75, 0.075, 0.3, "Euclidean dist: %2.4f" % peg_hole_euclidean_dist, color='blue')

            # write the frame to a file
            writer.grab_frame()

            if peg_hole_euclidean_dist <= 5e-3:
                print("Peg and hole are close enough to each other")
                break

            # Move the gripper along the Z-axis
            T_leftgripper[:3, 3] += lin_vel * T_leftgripper[:3, 2]
            T_peg[:3, 3] += lin_vel * T_peg[:3, 2]
            peg_hole_euclidean_dist = kinematic_utilities.norm(T_hole[:3, 3] - T_peg[:3, 3])
            print(peg_hole_euclidean_dist)

            # Hold the plot for 0.5s
            plt.pause(0.01)

            # Remove old peg plot
            h1.remove()
            peg_x.remove()
            peg_y.remove()
            peg_z.remove()
            itr_text.remove()
            dist_text.remove()

            # increament iteration counter
            itr += 1


# Shows peg-in-hole animation and plots peg-hole cross-section
def visualize_all(T_base2leftgripper, T_base2rightgripper, T_base2peg, T_base2hole, dist, P_hole2pegfinal):
    # See the animation
    view_animation(T_base2leftgripper, T_base2rightgripper, T_base2peg, T_base2hole, dist)

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
