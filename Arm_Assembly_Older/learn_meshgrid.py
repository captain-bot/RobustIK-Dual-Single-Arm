import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


sigma_list = [0.0020+0.0002*i for i in range(15)]
hole_radius_list = [0.027-0.0005*i for i in range(10)]

hole_radius_grid, sig_val_grid = np.meshgrid(hole_radius_list, sigma_list)
success_list = np.ones(hole_radius_grid.shape)

print(hole_radius_grid.shape)
print(sig_val_grid.shape)

# # 3D Plot
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.plot_surface(hole_radius_grid, sig_val_grid, np.array(success_list))
# plt.show()

hole_radius_grid_list = list(hole_radius_grid.reshape(hole_radius_grid.size))
print(hole_radius_grid_list)

# for i in range(1, 2):
#     print(i)
