import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

# import matplotlib.cm as cm
import matplotlib.animation as animation

garment_edge_size = 0.25
pile_height = 0

# Define waypoints (x, y, optional theta)
# waypoints = np.array([
#     [0, 0, 0],      # Start
#     [2, 1, np.pi/6],
#     [4, 3, np.pi/4],
#     [6, 5, np.pi/3],
#     [8, 6, np.pi/2]  # Goal
# ])
# waypoints = np.array([
#     [0.550703, 0.132448, 0.055, -178, -50, 13], # Grasp
#     [0.599305, 0.143997, 0.0548776],
#     [0.598393, 0.143835, 0.3],
#     [0.28, -0.291513, 0.4], #Check def
#     [0.12, -0.28, 0.400068],  # Place vertical
#     [0.120931, -0.279841, 0.055],
#     [0.0509354, -0.279833, 0.0559357],
#     [0.0517381, -0.27977, 0.35], #High pos
# ])

## Define waypoints (x, y, optional theta)
## Vertical placing
place_vertical = np.array([
    [0.28, 0.4, 0],                                             # Experiments2 (Check def)
    [0.12, 0.400068, 0], #[0.12, 0.400068, 0],                  # Pre place vertical
    [0.12, (pile_height+0.055), 0], #[0.120931, 0.055, 0],      # Place vertical
    [0.05, (pile_height+0.055), 0], #[0.0509354, 0.0559357, 0], # Post place
    [0.05, 0.35, 0] #[0.0517381, 0.35, 0]                       # High pos
])
## Diagonal placing
place_diagonal = np.array([
    [0.28, 0.4, 0],                                                                                                     # Experiments2 (Check def)
    [(garment_edge_size+0.12), garment_edge_size, 5*np.pi/3], #[0.37, 0.25, 5*np.pi/3],                                 # Pre place diagonal
    [(garment_edge_size+0.12)/2, ((garment_edge_size/2)+pile_height+0.06), 11*np.pi/6], #[0.185, 0.185, 11*np.pi/6],    # Place diagonal1
    [0.12, (pile_height+0.06), 0], #[0.12, 0.06, 0],                                                                    # Place diagonal2
    [0.05, (pile_height+0.06), 0], #[0.0500253, 0.0608235, 0],                                                          # Post place
    [0.05, 0.35, 0] #High pos
])
## Rotating placing
place_rot = np.array([
    [0.28, 0.4, 0], #Check def -0.291506,
    [0.45, garment_edge_size, 3*np.pi/2], #pre place rotating -0.28,
    [0.22, (pile_height+0.155), 11*np.pi/6], # place rotating -0.279989,
    [0.12, (pile_height+0.06), 0], #place222 -0.279994,
    [0.05, (pile_height+0.06), 0], #post place -0.279979,
    [0.05, 0.35, 0] #High pos -0.27985,
])
# ## Diagonal piling
# # waypoints = np.array([
# #     [0.28, 0.4], #Check def
# #     [0.37, 0.25], 
# #     [0.185, 0.252333],
# #     [0.12, 0.127333],
# #     [0.0500328, 0.128169],
# #     [0.0508214, 0.35] #High pos
# # ])
# # ## Diagonal piling
# # waypoints = np.array([
# #     [], #Check def
# #     [], 
# #     [],
# #     [],
# #     [] #High pos
# # ])
## Rotating piling
pile_rot = np.array([
    [0.28, 0.4, 0], #Check def -0.291506,
    [0.45, garment_edge_size, 3*np.pi/2], #pre place rotating -0.28,
    [(0.45-garment_edge_size), garment_edge_size, 11*np.pi/6], # piling -0.279989,
    [(0.45-garment_edge_size)-0.1, 0.105, 0], #piling2 -0.279994,
    [(0.45-garment_edge_size)-0.1-0.07, 0.105, 0], #post place -0.279979,
    [(0.45-garment_edge_size)-0.1, 0.35, 0] #High pos -0.27985,
])


# # Extract X, Y, and Theta values
# waypoints = place_vertical
# x = waypoints[:, 0]
# y = waypoints[:, 1]

# # Create a smooth parameterized curve using cubic splines
# t = np.linspace(0, 1, len(x))  # Normalized parameter
# cs_x = CubicSpline(t, x)  # X interpolation
# cs_y = CubicSpline(t, y)  # Y interpolation

# # Generate fine-grained trajectory points
# t_fine = np.linspace(0, 1, 100)
# x_smooth = cs_x(t_fine)
# y_smooth = cs_y(t_fine)

# # Plot the trajectory
# plt.figure(figsize=(8, 6))
# plt.plot(x, y, 'ro', label="Waypoints")  # Waypoints as red dots
# plt.plot(x_smooth, y_smooth, 'b-', label="Smooth Trajectory")  # Smooth curve

# # # Plot orientation (theta) as arrows
# # for i, (px, py, theta) in enumerate(waypoints):
# #     plt.arrow(px, py, 0.3 * np.cos(theta), 0.3 * np.sin(theta),
# #               head_width=0.2, head_length=0.2, fc='g', ec='g')

# plt.xlabel("X Position")
# plt.ylabel("Y Position")
# plt.title("Robotic Trajectory")
# plt.legend()
# plt.grid()
# plt.axis("equal")  # Keep aspect ratio
# plt.show()


##########FANCY VERSION


# # Define waypoints (x, y, optional theta)
# waypoints = np.array([
#     [0, 0, 0],      # Start
#     [2, 1, np.pi/6],
#     [4, 3, np.pi/4],
#     [6, 5, np.pi/3],
#     [8, 6, np.pi/2]  # Goal
# ])

# # Extract X, Y, and Theta values
# x = waypoints[:, 0]
# y = waypoints[:, 1]

# # Create a smooth parameterized curve using cubic splines
# t = np.linspace(0, 1, len(x))  # Normalized parameter
# cs_x = CubicSpline(t, x)  # X interpolation
# cs_y = CubicSpline(t, y)  # Y interpolation

# # Generate fine-grained trajectory points
# t_fine = np.linspace(0, 1, 100)
# x_smooth = cs_x(t_fine)
# y_smooth = cs_y(t_fine)

# # Color gradient for smooth trajectory
# colors = cm.plasma(np.linspace(0, 1, len(x_smooth)))

# # Create the figure
# fig, ax = plt.subplots(figsize=(8, 6), facecolor='black')
# ax.set_facecolor('black')
# ax.grid(color='gray', linestyle='--', linewidth=0.5)

# # Plot waypoints
# ax.scatter(x, y, c='white', edgecolors='black', s=100, label="Waypoints", zorder=3)

# # Plot smooth trajectory with color gradient
# for i in range(len(x_smooth) - 1):
#     ax.plot(x_smooth[i:i+2], y_smooth[i:i+2], color=colors[i], linewidth=2)

# # Plot orientation (theta) as arrows
# for px, py, theta in waypoints:
#     ax.arrow(px, py, 0.3 * np.cos(theta), 0.3 * np.sin(theta),
#              head_width=0.2, head_length=0.2, fc='cyan', ec='cyan')

# # Add labels and styling
# ax.set_xlabel("X Position", color='white')
# ax.set_ylabel("Y Position", color='white')
# ax.set_title("Fancy Robotic Trajectory", fontsize=14, color='white')
# ax.legend(loc="upper left", fontsize=10, facecolor='black', edgecolor='white')
# ax.axis("equal")

# # Animated robot marker
# robot_marker, = ax.plot([], [], 'ro', markersize=8, label="Robot")

# def update(frame):
#     robot_marker.set_data(x_smooth[frame], y_smooth[frame])
#     return robot_marker,

# # Create animation
# ani = animation.FuncAnimation(fig, update, frames=len(x_smooth), interval=50, blit=True)

# plt.show()


#############

### NOT SMOOTH

# Create the figure
fig, ax = plt.subplots(figsize=(8, 6), facecolor='white') #'black'
ax.set_facecolor('white') #black
ax.grid(color='gray', linestyle='--', linewidth=0.5)


trajectories = [place_vertical, place_diagonal, place_rot, pile_rot] # Trajectories to plot
names = ['Vertical', 'Diagonal', 'Rotating', 'Pile rotating']
colors = ['cyan', 'green', 'red', 'yellow']
i=0
for data in trajectories:
    waypoints = data
    color = colors[i]
    name = names[i]
    x = waypoints[:, 0]
    y = waypoints[:, 1]
    ax.scatter(x, y, c='white', edgecolors='black', s=100, zorder=3) # Plot waypoints as white dots
    ax.plot(x, y, linestyle='-', linewidth=2, color=color, label=name) # Plot **straight-line** connections between waypoints
    # Plot orientation (theta) as arrows
    for px, py, theta in waypoints:
        ax.arrow(px, py, 0.05 * np.cos(theta), 0.05 * np.sin(theta),
                head_width=0.01, head_length=0.01, fc='yellow', ec='yellow')
    i+=1


ax.invert_xaxis()

# Add labels and styling
ax.set_xlabel("X Position", color='black')
ax.set_ylabel("Y Position", color='black')
ax.set_title("Waypoint Trajectory with Straight Lines", fontsize=14, color='black')
ax.legend(loc="upper left", fontsize=10, facecolor='white', edgecolor='black')
ax.axis("equal")

# Animated robot marker
robot_marker, = ax.plot([], [], 'ro', markersize=8, label="Robot")

def update(frame):
    robot_marker.set_data(x[frame], y[frame])
    return robot_marker,

# Create animation (robot moves waypoint by waypoint)
ani = animation.FuncAnimation(fig, update, frames=len(x), interval=500, blit=True)

plt.show()
