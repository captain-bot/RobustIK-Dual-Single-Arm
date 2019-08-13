import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')


def peg_cross_section():
    T_base2leftgripper = np.array([[-0.97605573, - 0.2120826, - 0.04510196,  0.75462528],
                                   [0.05402458, - 0.03645633, - 0.99772255,  0.22282825],
                                   [0.21001876, - 0.97656434, 0.0470553,  0.44900097],
                                   [0.0,          0.0,          0.0,          1.0]])

    for _ in range(100):
        pt_x = np.linspace(-0.010, 0.010, 20)
        pt_y = np.linspace(-0.010, 0.010, 20)
        x_grid, y_grid = np.meshgrid(pt_x, pt_y)
        z_grid = 0.050*np.ones(x_grid.shape)

        pts = np.vstack((x_grid.reshape(1, x_grid.size), y_grid.reshape(1, y_grid.size), z_grid.reshape(1, z_grid.size)))
        pts = np.dot(T_base2leftgripper[:3, :3], pts) + T_base2leftgripper[:3, 3].reshape(3, 1)

        x_grid = pts[0, :].reshape(20, 20)
        y_grid = pts[1, :].reshape(20, 20)
        z_grid = pts[2, :].reshape(20, 20)
        peg_face = ax.plot_surface(x_grid, y_grid, z_grid, alpha=0.5, color='r')

        # Compute peg tip frame
        T_peg = T_base2leftgripper.copy()
        T_peg[:3, 3] += 0.050 * T_peg[:3, 2]

        peg_x = ax.quiver(T_peg[0, 3], T_peg[1, 3], T_peg[2, 3], T_peg[0, 0], T_peg[1, 0],
                  T_peg[2, 0], length=0.01, normalize=False, color='r')
        peg_y = ax.quiver(T_peg[0, 3], T_peg[1, 3], T_peg[2, 3], T_peg[0, 1], T_peg[1, 1],
                  T_peg[2, 1], length=0.01, normalize=False, color='g')
        peg_z = ax.quiver(T_peg[0, 3], T_peg[1, 3], T_peg[2, 3], T_peg[0, 2], T_peg[1, 2],
                  T_peg[2, 2], length=0.01, normalize=False, color='b')

        # Move left gripper frame
        T_base2leftgripper[:3, 3] += 0.002*T_base2leftgripper[:3, 2]

        # Hold the plot for 0.5s
        plt.pause(0.01)

        peg_face.remove()
        peg_x.remove()
        peg_y.remove()
        peg_z.remove()


def hole_cross_section():
    T_base2rightgripper = np.array([[-0.97588953,  0.21245144,  0.04692548,  0.74543445],
                                    [0.05553813,  0.03474443,  0.99770053,  0.0225008],
                                    [0.21039606, 0.97654657, - 0.04571969, 0.45900191],
                                    [0.0,          0.0,          0.0,          1.0]])

    pt_x = np.linspace(-0.015, 0.015, 20)
    pt_y = np.linspace(-0.015, 0.015, 20)
    x_grid, y_grid = np.meshgrid(pt_x, pt_y)
    z_grid = 0.050*np.ones(x_grid.shape)

    pts = np.vstack((x_grid.reshape(1, x_grid.size), y_grid.reshape(1, y_grid.size), z_grid.reshape(1, z_grid.size)))
    pts = np.dot(T_base2rightgripper[:3, :3], pts) + T_base2rightgripper[:3, 3].reshape(3, 1)

    x_grid = pts[0, :].reshape(20, 20)
    y_grid = pts[1, :].reshape(20, 20)
    z_grid = pts[2, :].reshape(20, 20)
    ax.plot_surface(x_grid, y_grid, z_grid, alpha=0.5, color='b')

    # Compute hole tip frame
    T_hole = T_base2rightgripper.copy()
    T_hole[:3, 3] += 0.050*T_hole[:3, 2]

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

    ax.set_xlim(0.65, 0.78)
    ax.set_ylim(0.06, 0.18)
    ax.set_zlim(0.4, 0.5)
    ax.view_init(36, 154)


# Plot the surfaces
hole_cross_section()
peg_cross_section()
# plt.show()
