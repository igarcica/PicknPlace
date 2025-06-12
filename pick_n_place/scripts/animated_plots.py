### Para borrar archivo
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.interpolate import make_interp_spline

###################### PLACING QUALITY EVOLUTION

# Data
# y = np.array([0, 13, 49, 88, 94, 65, 56, 94, 97, 91, 90, 95, 96, 83, 86, 89, 96, 94, 88, 94, 92, 98, 99, 96, 99, 99, 98])
y = np.array([0, 0, 0, 8, 31, 81, 86, 78, 78, 93, 78, 79, 85, 93, 77, 88, 93, 91, 85, 92, 91, 76, 88, 91, 90, 94, 95])
x = np.linspace(0, 27, len(y))
print(x)

# Interpolation
x_smooth = np.linspace(x.min(), x.max(), 500)
spline = make_interp_spline(x, y, k=3)
y_smooth = spline(x_smooth)

# Setup
fig, ax = plt.subplots(figsize=(10, 6))
line, = ax.plot([], [], lw=2, color='#1f77b4')
point, = ax.plot([], [], 'ro')
scatter = ax.scatter([], [], color='black', s=50, zorder=3)  # Hollow circles

ax.set_xlim(x.min(), x.max())
ax.set_ylim(y.min() - 10, y.max() + 10)

plt.axvline(x=20.76923077, color='gray', linestyle='--', linewidth=2) #Plot a dashed line when plotting the pillowcase + towel results

# Styling
ax.set_xlabel("Trial", fontsize=18)
ax.set_ylabel("Pile quality (%)", fontsize=18)
ax.set_title("Pile quality evolution over trials", fontsize=20, fontweight='bold', color="#333333")
ax.grid(True, linewidth=0.8, alpha=0.5)
ax.spines["top"].set_visible(False)
ax.spines["right"].set_visible(False)

# Example custom labels (must match length of x)
custom_labels = [f"{i}" for i in range(len(x))]  # e.g., T0, T1, ..., T26
ax.set_xticks(x) # Set ticks at original data point positions
ax.set_xticklabels(custom_labels, fontsize=10)  # Rotate for readability


# Animation
def init_plot_quality_anim():
    line.set_data([], [])
    point.set_data([], [])
    scatter.set_offsets([])
    return line, point, scatter

def update_plot_quality_anim(frame):
    # Curve line + red dot
    line.set_data(x_smooth[:frame], y_smooth[:frame])
    point.set_data(x_smooth[frame - 1], y_smooth[frame - 1])

    # Show markers up to the current x_smooth point
    current_x = x_smooth[frame - 1]
    visible_indices = np.where(x <= current_x)[0]
    scatter.set_offsets(np.column_stack((x[visible_indices], y[visible_indices])))

    return line, point, scatter

# Animate
ani = FuncAnimation(fig, update_plot_quality_anim, frames=len(x_smooth), init_func=init_plot_quality_anim,
                    blit=False, interval=15, repeat=False)

plt.tight_layout()

# Save as video
ani.save("pile_quality_animation.mp4", writer="ffmpeg", fps=30)
plt.show()



###################### COST EVOLUTION


# # Matrix evolution (each step is a 3x3 array)
# matrices = [
#     np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]]),
#     np.array([[30, 0, 0], [0, 0, 0], [0, 0, 0]]),
#     np.array([[30, 30, 0], [0, 0, 0], [0, 0, 0]])
# ]

# # Create plot
# fig, ax = plt.subplots()
# heatmap = ax.imshow(matrices[0], cmap='YlOrRd', vmin=0, vmax=50)

# # Add color bar
# cbar = plt.colorbar(heatmap, ax=ax)
# cbar.set_label("Value", fontsize=12)

# # Axis styling
# ax.set_xticks(np.arange(3))
# ax.set_yticks(np.arange(3))
# ax.set_xticklabels(['Vertical', 'Diagonal', 'Rotating'])
# ax.set_yticklabels(['A', 'B', 'C'])
# ax.set_title("Cost Evolution", fontsize=20, fontweight='bold')
# ax.set_xlabel("Action (Placement)", fontsize=18)
# ax.set_ylabel("State (Deformation class)", fontsize=18)


# # Optional: grid
# for edge, spine in ax.spines.items():
#     spine.set_visible(False)
# ax.set_xticks(np.arange(-.5, 3, 1), minor=True)
# ax.set_yticks(np.arange(-.5, 3, 1), minor=True)
# ax.grid(which='minor', color='gray', linestyle='-', linewidth=1)
# ax.tick_params(which='minor', bottom=False, left=False)

# # Initialize cell labels
# nrows, ncols = matrices[0].shape
# cell_texts = [[ax.text(j, i, "", ha="center", va="center", color="black", fontsize=12)
#                for j in range(ncols)] for i in range(nrows)]

# # Animation update
# def update(frame):
#     heatmap.set_data(matrices[frame])
#     return [heatmap]

# # Animate
# ani = FuncAnimation(fig, update, frames=len(matrices), interval=1000, repeat=False)

# plt.tight_layout()
# plt.show()


# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation

# # cloth-to-cloth costs
# matrices = [
#     np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]]),
#     np.array([[0, 0, 0], [0, 0, 0], [30, 0, 0]]),
#     np.array([[0, 0, 0], [0, 0, 0], [30, 30, 0]]),
#     np.array([[0, 0, 0], [0, 0, 0], [30, 30, 30]]),
#     np.array([[0, 0, 0], [30, 0, 0], [30, 30, 30]]),
#     np.array([[0, 0, 0], [30, 9, 0], [30, 30, 30]]),
#     np.array([[0, 0, 0], [30, 9, 7], [30, 30, 30]]),
#     np.array([[0, 0, 0], [30, 9, 11], [30, 30, 30]]),
#     np.array([[0, 0, 0], [30, 12, 11], [30, 30, 30]]),
#     np.array([[0, 0, 0], [30, 12, 9], [30, 30, 30]]),
#     np.array([[0, 0, 0], [30, 12, 12], [30, 30, 30]]),
#     np.array([[0, 0, 0], [30, 14, 12], [30, 30, 30]]),
#     np.array([[0, 0, 0], [30, 14, 12], [30, 30, 30]]),
#     np.array([[0, 0, 0], [30, 14, 10], [30, 30, 30]]),
#     np.array([[0, 0, 0], [30, 14, 13], [30, 30, 30]]),
#     np.array([[0, 0, 0], [30, 14, 12], [30, 30, 30]]),
#     np.array([[0, 0, 0], [30, 14, 10], [30, 30, 30]]),
#     np.array([[0, 0, 0], [30, 14, 9], [30, 30, 30]]),
#     np.array([[0, 0, 0], [30, 14, 10], [30, 30, 30]]),
#     np.array([[0, 0, 0], [30, 14, 9], [30, 30, 30]]),
#     np.array([[0, 0, 0], [30, 14, 9], [30, 30, 30]]), #trial21
#     np.array([[7, 0, 0], [30, 14, 9], [30, 30, 30]]),
#     np.array([[7, 3, 0], [30, 14, 9], [30, 30, 30]]),
#     np.array([[7, 3, 2], [30, 14, 9], [30, 30, 30]]),
#     np.array([[7, 3, 4], [30, 14, 9], [30, 30, 30]]),
#     np.array([[7, 3, 4], [30, 14, 9], [30, 30, 30]]),
#     np.array([[7, 3, 4], [30, 14, 9], [30, 30, 30]]) #trial 27
# ]

# # cloth-to-table costs
# matrices2 = [
#     np.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]]),
#     np.array([[0, 0, 0], [0, 0, 0], [30, 0, 0]]),
#     np.array([[0, 0, 0], [0, 0, 0], [30, 25, 0]]),
#     np.array([[0, 0, 0], [0, 0, 0], [30, 25, 6]]),
#     np.array([[0, 0, 0], [3, 0, 0], [30, 25, 6]]),
#     np.array([[17, 0, 0], [3, 0, 0], [30, 25, 6]]),
#     np.array([[17, 0, 0], [3, 22, 0], [30, 25, 6]]),
#     np.array([[17, 0, 0], [3, 22, 1], [30, 25, 6]]),
#     np.array([[17, 0, 0], [3, 22, 1], [30, 25, 6]]),
#     np.array([[17, 0, 0], [3, 22, 3], [30, 25, 6]]),
#     np.array([[17, 0, 0], [5, 22, 3], [30, 25, 6]]),
#     np.array([[17, 0, 0], [5, 22, 3], [30, 25, 6]]),
#     np.array([[17, 0, 0], [5, 22, 3], [30, 25, 6]]),
#     np.array([[17, 0, 0], [5, 22, 7], [30, 25, 6]]),
#     np.array([[17, 0, 0], [7, 22, 7], [30, 25, 6]]),
#     np.array([[17, 0, 0], [8, 22, 7], [30, 25, 6]]),
#     np.array([[17, 0, 0], [8, 22, 6], [30, 25, 6]]),
#     np.array([[17, 0, 0], [8, 22, 6], [30, 25, 6]]),
#     np.array([[17, 0, 0], [8, 22, 7], [30, 25, 6]]),
#     np.array([[17, 0, 0], [8, 22, 6], [30, 25, 6]]),
#     np.array([[17, 0, 0], [8, 22, 6], [30, 25, 6]]), #trial 21
#     np.array([[17, 1, 0], [8, 22, 6], [30, 25, 6]]),
#     np.array([[17, 1, 0], [8, 22, 6], [30, 25, 6]]),
#     np.array([[17, 1, 1], [8, 22, 6], [30, 25, 6]]),
#     np.array([[17, 1, 1], [8, 22, 6], [30, 25, 6]]),
#     np.array([[17, 1, 1], [8, 22, 6], [30, 25, 6]]),
#     np.array([[17, 1, 1], [8, 22, 6], [30, 25, 6]])
# ]

# matrices=matrices2
# nrows, ncols = matrices[0].shape
# print(len(matrices))

# # Create plot
# fig, ax = plt.subplots()
# heatmap = ax.imshow(matrices[0], cmap='YlOrRd', vmin=0, vmax=50)

# # Color bar
# cbar = plt.colorbar(heatmap, ax=ax)
# cbar.set_label("Value", fontsize=12)

# # Axis styling
# ax.set_xticks(np.arange(ncols))
# ax.set_yticks(np.arange(nrows))
# ax.set_xticklabels(['Vertical', 'Diagonal', 'Rotating'])
# ax.set_yticklabels(['A', 'B', 'C'])
# ax.set_title("Cost Evolution", fontsize=16, fontweight='bold')

# # Grid lines
# for edge, spine in ax.spines.items():
#     spine.set_visible(False)
# ax.set_xticks(np.arange(-.5, ncols, 1), minor=True)
# ax.set_yticks(np.arange(-.5, nrows, 1), minor=True)
# ax.grid(which='minor', color='gray', linestyle='-', linewidth=1)
# ax.tick_params(which='minor', bottom=False, left=False)

# # Initialize cell labels
# cell_texts = [[ax.text(j, i, "", ha="center", va="center", color="black", fontsize=12)
#                for j in range(ncols)] for i in range(nrows)]

# # Animation update
# def update(frame):
#     data = matrices[frame]
#     heatmap.set_data(data)
#     for i in range(nrows):
#         for j in range(ncols):
#             cell_texts[i][j].set_text(f"{data[i, j]}")
#     return [heatmap] + [text for row in cell_texts for text in row]

# # Animate
# ani = FuncAnimation(fig, update, frames=len(matrices), interval=1000, repeat=False)

# # plt.tight_layout()
# ani.save("cost_update.mp4", writer="ffmpeg", fps=30)
# plt.show()



