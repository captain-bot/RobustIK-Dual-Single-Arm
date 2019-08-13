import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl

fig, ax = plt.subplots(1)

rect_hole = patches.Rectangle((-0.025, -0.025), 0.050, 0.050, linewidth=1, alpha=0.3, edgecolor='b', facecolor='b')
rect_peg = patches.Rectangle((-0.0255, -0.0195), 0.045, 0.045, linewidth=1, alpha=0.3, edgecolor='r', facecolor='r')

t2 = mpl.transforms.Affine2D().rotate_deg(2.7) + ax.transData
rect_peg.set_transform(t2)

ax.add_patch(rect_hole)
ax.add_patch(rect_peg)

ax.set_xlabel("hole_x [m]")
ax.set_xlabel("hole_y [m]")
ax.set_xlim(-0.030, 0.030)
ax.set_ylim(-0.030, 0.030)
ax.set_aspect('equal')
ax.set_title("Peg and hole orientation in hole XY plane")
plt.show()
