import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline


#### PLOT PLACING QUALITY TENDENCY (placed and piled quality)

placed_quality_results = np.array([0, 13, 49, 88, 91, 83, 91, 88, 92, 78])

piled_quality_results = np.array([0, 0, 0, 8, 42, 43, 59, 0, 33, 28])


# Create a smooth parameterized curve using cubic splines
x = np.arange(1, len(placed_quality_results) + 1) #np.linspace(0,1, len(x))
t = np.linspace(0, 1, len(x))  # Normalized parameter
cs_x = CubicSpline(t, x)  # X interpolation
cs_yplaced = CubicSpline(t, placed_quality_results)  # Y interpolation
cs_ypiled = CubicSpline(t, piled_quality_results)  # Y interpolation

# Generate fine-grained trajectory points
t_fine = np.linspace(0, 1, 100)
x_smooth = cs_x(t_fine)
yplaced_smooth = cs_yplaced(t_fine)
ypiled_smooth = cs_ypiled(t_fine)

# Plot the trajectory
plt.figure(figsize=(8, 6))
plt.plot(x, placed_quality_results, 'ro')  # Waypoints as red dots
plt.plot(x_smooth, yplaced_smooth, 'g-', label="Placed quality")  # Smooth curve
plt.plot(x, piled_quality_results, 'ro')  # Waypoints as red dots
plt.plot(x_smooth, ypiled_smooth, 'b-', label="Pile quality")  # Smooth curve

plt.xticks(x)
plt.xlabel("Experiment")
plt.ylabel("Placing quality")
plt.title("Placing quality through experiments")
plt.legend()
plt.grid()
plt.show()


##############################################
#### PLOT CHANGES IN COST TABLE
## ---- Placing cost table ----
# Def clas | Vertical | Diagonal | Rotating
#    A     |    0     |    0     |    0
#    B     |    0     |    0     |    0
#    C     |    0     |    0     |    0
# experiments = np.arange(1, 5)
n_exp = 10
## ---- PLACE cost table ----
casilla_A_v = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
casilla_C_v = [0, 15, 15, 15, 15, 15, 15, 15, 15, 15]
casilla_C_d = [0, 0, 15, 15, 15, 15, 15, 15, 15, 15]
casilla_C_r = [0, 0, 0, 6, 7, 12, 10, 11, 9, 15]
## ---- PILE cost table ----
p_casilla_A_v = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
p_casilla_C_v = [0, 15, 15, 15, 30, 30, 30, 30, 45, 45]
p_casilla_C_d = [0, 0, 15, 15, 15, 30, 30, 30, 30, 45]
p_casilla_C_r = [0, 0, 0, 15, 15, 15, 28, 43, 43, 43]

experiments = np.linspace(0, 1, n_exp)
# casillas_place = [casilla_A_v, casilla_A_d, casilla_A_r, casilla_B_v, casilla_B_d, casilla_B_r, casilla_C_v, casilla_C_d, casilla_C_r] 
casillas_place = [casilla_C_v, casilla_C_d, casilla_C_r]
casillas_pile = [p_casilla_C_v, p_casilla_C_d, p_casilla_C_r]
names = ['C vertical', 'C diagonal', 'C rotating']
colors = ['cyan', 'green', 'red']

# Create a smooth parameterized curve using cubic splines
x = np.arange(1, len(experiments)+1)
t = np.linspace(0, 1, len(x))  # Normalized parameter
cs_x = CubicSpline(t, x)  # X interpolation
# Generate fine-grained trajectory points
t_fine = np.linspace(0, 1, 100)
x_smooth = cs_x(t_fine)
plt.figure(figsize=(8, 6))

## Plot place cost table updates
i=0
for casilla in casillas_place:
    data = casilla
    name = names[i]
    color = colors[i]
    cs_y = CubicSpline(t, data) 
    y_smooth = cs_y(t_fine)
    plt.plot(x, data, 'bo', linestyle='-', linewidth=2, color=color, label=name)  # Waypoints as red dots
    # plt.plot(x_smooth, y_smooth, 'g-', label="A vertical")  # Smooth curve
    i+=1
## Plot config
plt.xticks(x)
plt.xlabel("Experiment")
plt.ylabel("Current cost (placing error)")
plt.title("PLACE Cost update evolution")
plt.legend()
plt.grid()

#Plot PILE 
plt.figure(figsize=(8, 6))
i=0
for casilla in casillas_pile:
    data = casilla
    name = names[i]
    color = colors[i]
    cs_y = CubicSpline(t, data) 
    y_smooth = cs_y(t_fine)
    plt.plot(x, data, 'bo', linestyle='-', linewidth=2, color=color, label=name)  # Waypoints as red dots
    # plt.plot(x_smooth, y_smooth, 'g-', label="A vertical")  # Smooth curve
    i+=1

## Plot config
plt.xticks(x)
plt.xlabel("Experiment")
plt.ylabel("Current cost (placing error)")
plt.title("PILE Cost update evolution")
plt.legend()
plt.grid()
plt.show()


# ## Plot data
# data = casilla_A_d
# cs_y = CubicSpline(t, data) 
# y_smooth = cs_y(t_fine)
# plt.plot(x, data, 'ro', linestyle='-', linewidth=2, color='cyan', label="A diagonal")  # Waypoints as red dots
# # plt.plot(x_smooth, y_smooth, 'g-', label="A vertical")  # Smooth curve