import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline


#### PLOT PLACING QUALITY TENDENCY (placed and piled quality)

placed_quality_results = np.array([
    0, 
    80,
    100,
    80
])

piled_quality_results = np.array([
    0, 
    70,
    80,
    60
])


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
experiments = np.arange(1, 5)
casilla_A_v = [0, 20, 20, 20]
casilla_A_d = [0, 0, 10, 10]
casilla_A_r = [0, 0, 0, 20]
casilla_B_v = [0, 0, 0, 0]

# Create a smooth parameterized curve using cubic splines
x = np.arange(1, len(experiments)+1)
t = np.linspace(0, 1, len(x))  # Normalized parameter
cs_x = CubicSpline(t, x)  # X interpolation
# Generate fine-grained trajectory points
t_fine = np.linspace(0, 1, 100)
x_smooth = cs_x(t_fine)
plt.figure(figsize=(8, 6))

## Plot data
data = casilla_A_v
cs_y = CubicSpline(t, data) 
y_smooth = cs_y(t_fine)
plt.plot(x, data, 'bo', linestyle='-', linewidth=2, color='green', label="A Vertical")  # Waypoints as red dots
# plt.plot(x_smooth, y_smooth, 'g-', label="A vertical")  # Smooth curve

## Plot data
data = casilla_A_d
cs_y = CubicSpline(t, data) 
y_smooth = cs_y(t_fine)
plt.plot(x, data, 'ro', linestyle='-', linewidth=2, color='cyan', label="A diagonal")  # Waypoints as red dots
# plt.plot(x_smooth, y_smooth, 'g-', label="A vertical")  # Smooth curve

## Plot config
plt.xticks(x)
plt.xlabel("Experiment")
plt.ylabel("Placing quality")
plt.title("Placing quality through experiments")
plt.legend()
plt.grid()
plt.show()