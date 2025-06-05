
####This script serves to plot the placing quality and cost evolution over time.
## Costs values are obtained with placing_costs_update.py
## SCRIPT TO DELETE, NEW VERSION IN main_cost_update.py

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline


#### PLOT PLACING QUALITY TENDENCY (placed and piled quality)

# placed_quality_results = np.array([0, 13, 49, 88, 91, 83, 91, 88, 92, 78])
# piled_quality_results = np.array([0, 0, 0, 8, 42, 43, 59, 0, 33, 28])

# ####### NEW PILLOWCASE ORDER (with short edge)
placed_quality_results = np.array([0, 13, 49, 88, 94, 65, 51, 94, 97, 91])
piled_quality_results = np.array([0, 0, 0, 8, 31, 81, 86, 78, 78, 93])
strategies = np.array(["0", "vv", "dd", "rr", "vv", "vd", "dr", "rr", "rd", "rr"])

#### TOWEL ####
# placed_quality_results = np.array([0, 92, 92, 94, 96])
# piled_quality_results = np.array([0, 70, 89, 80, 91])


# Create a smooth parameterized curve using cubic splines
x = np.arange(0, len(placed_quality_results)) #np.linspace(0,1, len(x))
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

# for i in strategies:
#     plt.annotate(strategies[i], (x,placed_quality_results))
for i,j in zip(x,placed_quality_results):
    plt.annotate(strategies[i], (i+0.05,j+0.05))

plt.xticks(x)
plt.xlabel("Experiment")
plt.ylabel("Placing quality")
plt.title("Placing quality through experiments")
plt.legend()
plt.grid()
# plt.show()


##############################################
#### PLOT CHANGES IN COST TABLE
## ---- Placing cost table ----
# Def clas | Vertical | Diagonal | Rotating
#    A     |    0     |    0     |    0
#    B     |    0     |    0     |    0
#    C     |    0     |    0     |    0
# experiments = np.arange(1, 5)
n_exp = 10

# ## ---- PLACE cost table ----
# casilla_A_v = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# casilla_C_v = [0, 15, 15, 15, 15, 15, 15, 15, 15, 15]
# casilla_C_d = [0, 0, 15, 15, 15, 15, 15, 15, 15, 15]
# casilla_C_r = [0, 0, 0, 6, 7, 12, 10, 11, 9, 15]
# ## ---- PILE cost table ----
# p_casilla_A_v = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# p_casilla_C_v = [0, 15, 15, 15, 30, 30, 30, 30, 45, 45]
# p_casilla_C_d = [0, 0, 15, 15, 15, 30, 30, 30, 30, 45]
# p_casilla_C_r = [0, 0, 0, 15, 15, 15, 28, 43, 43, 43]

# ##### NEW PILLOWCASE ORDER (with short edge) - exps:1,2,3,4,5,6,15,16, next br+br / decaying alfa + c=30 /  CostUpdater(0.5, 0.3, 30, 0.5)
# ## ---- PLACE cost table ----
# casilla_A_v = [0, 0, 0, 0, 0, 15, 15, 15, 15]
# casilla_B_v = [0, 0, 0, 0, 3, 3, 3, 3, 3]
# casilla_B_d = [0, 0, 0, 0, 0, 0, 15, 15, 15]
# casilla_B_r = [0, 0, 0, 0, 0, 0, 0, 2, 2]
# casilla_C_v = [0, 15, 15, 15, 15, 15, 15, 15, 15]
# casilla_C_d = [0, 0, 15, 15, 15, 15, 15, 15, 15]
# casilla_C_r = [0, 0, 0, 6, 6, 6, 6, 6, 6]
# ## ---- PILE cost table ----
# p_casilla_A_v = [0, 0, 0, 0, 0, 0, 0, 0, 0]
# p_casilla_B_v = [0, 0, 0, 0, 15, 15, 15, 15, 15]
# p_casilla_B_d = [0, 0, 0, 0, 0, 9, 9, 9, 15]
# p_casilla_B_r = [0, 0, 0, 0, 0, 0, 7, 14, 14]
# p_casilla_C_v = [0, 15, 15, 15, 15, 15, 15, 15, 15]
# p_casilla_C_d = [0, 0, 15, 15, 15, 15, 15, 15, 15]
# p_casilla_C_r = [0, 0, 0, 15, 15, 15, 15, 15, 15]

##### NEW PILLOWCASE ORDER (with short edge) - exps:1,2,3,4,5,6,15,16, next br+br / decaying alfa + c=30 /  CostUpdater(0.5, 0.3, 45, 0.5)
## ---- PLACE cost table ----
casilla_A_v = [0, 0, 0, 0, 0, 17, 17, 17, 17, 17]
casilla_B_v = [0, 0, 0, 0, 3, 3, 3, 3, 3, 3]
casilla_B_d = [0, 0, 0, 0, 0, 0, 22, 22, 22, 33]
casilla_B_r = [0, 0, 0, 0, 0, 0, 0, 2, 2, 5]
casilla_C_v = [0, 22, 22, 22, 22, 22, 22, 22, 22, 22]
casilla_C_d = [0, 0, 22, 22, 22, 22, 22, 22, 22, 22]
casilla_C_r = [0, 0, 0, 6, 6, 6, 6, 6, 6, 6]
## ---- PILE cost table ----
p_casilla_A_v = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
p_casilla_B_v = [0, 0, 0, 0, 22,22,22,22,22,22]
p_casilla_B_d = [0, 0, 0, 0, 0, 9, 9, 9, 15, 15]
p_casilla_B_r = [0, 0, 0, 0, 0, 0, 7, 14, 14, 10]
p_casilla_C_v = [0, 22,22,22,22,22,22,22,22,22]
p_casilla_C_d = [0, 0, 22,22,22,22,22,22,22,22]
p_casilla_C_r = [0, 0, 0, 22,22,22,22,22,22,22]


# ######## TOWEL ########
# ## ---- PLACE cost table ----
# t_casilla_A_v = [0, 4, 4, 4, 4]
# t_casilla_A_d = [0, 0, 4, 4, 4]
# t_casilla_A_r = [0, 0, 0, 3, 3]
# ## ---- PILE cost table ----
# t_p_casilla_A_v = [0, 15, 15, 15, 15]
# t_p_casilla_A_d = [0, 0, 5, 5, 7]
# t_p_casilla_A_r = [0, 0, 0, 10, 10]


experiments = np.linspace(0, 1, n_exp)
casillas_place = [casilla_A_v, casilla_B_v, casilla_B_d, casilla_B_r, casilla_C_v, casilla_C_d, casilla_C_r] 
casillas_pile = [p_casilla_A_v, p_casilla_B_v, p_casilla_B_d, p_casilla_B_r, p_casilla_C_v, p_casilla_C_d, p_casilla_C_r] 
# casillas_place = [casilla_C_v, casilla_C_d, casilla_C_r]
# casillas_pile = [p_casilla_C_v, p_casilla_C_d, p_casilla_C_r]
######### TOWEL 
# casillas_place = [t_casilla_A_v, t_casilla_A_d, t_casilla_A_r]
# casillas_pile = [t_p_casilla_A_v, t_p_casilla_A_d, t_p_casilla_A_r]

names = ['A vertical', 'B vertical', 'B diagonal', 'B rotating', 'C vertical', 'C diagonal', 'C rotating']
colors = ['black', 'cyan', 'green', 'pink', 'blue', 'yellow', 'red']
# names = ['C vertical', 'C diagonal', 'C rotating']
# colors = ['cyan', 'green', 'red']

# Create a smooth parameterized curve using cubic splines
x = np.arange(0, len(experiments))
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
    # plt.plot(x, placed_quality_results, 'ro')  # Waypoints as red dots
    # plt.plot(x_smooth, y_smooth, 'g-', label="A vertical")  # Smooth curve
    i+=1

## Plot config
plt.xticks(x)
plt.xlabel("Experiment")
plt.ylabel("Current cost (placing error)")
plt.title("PLACE Cost update evolution - 0.5, 0.3, 45, 0.5")
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
plt.title("PILE Cost update evolution - 0.5, 0.3, 30, 0.5")
plt.legend()
plt.grid()
plt.show()


# ## Plot data
# data = casilla_A_d
# cs_y = CubicSpline(t, data) 
# y_smooth = cs_y(t_fine)
# plt.plot(x, data, 'ro', linestyle='-', linewidth=2, color='cyan', label="A diagonal")  # Waypoints as red dots
# # plt.plot(x_smooth, y_smooth, 'g-', label="A vertical")  # Smooth curve