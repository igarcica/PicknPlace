### This script collects the placing_costs_update.py and the plot_placing_quality.py

import numpy as np
import statistics as sts
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

####################################################################################
### INPUT DATA

#### PILLOWCASE ####
placed_quality_results = np.array([0, 13, 49, 88, 94, 65, 56, 94, 97, 91, 90, 95, 96, 83, 86, 89, 94]) #exp31(t=10) bit changed
placing_errors = 100-placed_quality_results
placing_str = ["0", "v", "d", "r", "v", "v", "d", "r", "r", "r", "v", "r", "r", "r", "v", "v", "r"]
placing_def_classes = ["0", "C", "C", "C", "B", "A", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B"]

piled_quality_results = np.array([0, 0, 0, 8, 31, 81, 86, 78, 78, 93, 78, 79, 85, 93, 77, 88, 91]) #exp31(t=10) bit changed
piling_errors = 100-piled_quality_results
piling_str = ["0", "v", "d", "r", "v", "d", "r", "r", "d", "r", "r", "d", "r", "r", "r", "r", "r"] 
piling_def_classes = ["0", "C", "C", "C", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B"]

labels = np.array(["0", "vv", "dd", "rr", "vv", "vd", "dr", "rr", "rd", "rr", "vr", "rd", "rr", "rr", "vr", "vr", "rr"])

### TOWEL ####
# placed_quality_results = np.array([0, 92, 92, 94, 96]) 
# placing_errors = 100-placed_quality_results
# placing_str = ["0", "v", "d", "r", "r"]
# placing_def_classes = ["0", "A", "A", "A", "A"]
# piled_quality_results = np.array([0, 70, 89, 80, 91]) 
# piling_errors = 100-piled_quality_results
# piling_str = ["0", "v", "d", "r", "d"] 
# piling_def_classes = ["0", "A", "A", "A", "A"]

# labels = np.array(["0", "vv", "dd", "rr", "rd"])

#### PILLOWCASE ONLY LONG EDGE ####
# placed_quality_results = np.array([0, 13, 49, 88, 91, 83, 91,88, 78, 92]) 
# placing_errors = 100-placed_quality_results
# placing_str = ["0", "v", "d", "r", "r", "r", "r", "r", "r", "r"]
# placing_def_classes = ["0", "C", "C", "C", "C", "C", "C", "C", "C", "C"]
# piled_quality_results = np.array([0, 0,0,8,42,43,59, 0, 22, 33]) 
# piling_errors = 100-piled_quality_results
# piling_str = ["0", "v", "d", "r", "v", "d", "r", "r", "d", "v"] 
# piling_def_classes = ["0", "C", "C", "C", "C", "C", "C", "C", "C", "C"]
# labels = np.array(["0", "vv", "dd", "rr", "rv", "rd", "rr", "rr", "rd", "rv"])

errors = [placing_errors, piling_errors]
strategies = [placing_str, piling_str]
classes = [placing_def_classes, piling_def_classes]



delta=45
# print("Std deviation: ", sts.stdev(placing_errors))
# print("Std deviation: ", sts.stdev(piling_errors))


####################################################################################
class CostUpdater:
    def __init__(self, cost_table, alpha_0=1, alpha_stab=0.3, c=30, beta=1):
        """
        Initialize with:
        - cost_table: initial cost matrix (numpy array)
        - alpha: learning rate for updates
        - c: threshold for Huber loss
        """
        self.cost_table = cost_table
        self.alpha_0 = alpha_0
        self.alpha_stab = alpha_stab
        self.c = c
        self.beta = beta                #beta 1.5 stabilize alpha at step 6
        self.cells = [[[] for _ in range(3)] for _ in range(3)] # Create a storage list for each cell

    def get_alpha(self, t):
        """Compute decayed alpha based on the selected decay type."""
        # self.alpha = self.alpha_0 # Fized alpha
        # alpha = self.alpha_0 + (1-t)*t #en pruebas
        # alpha = self.alpha_stab + (self.alpha_0-self.alpha_stab)/(1+np.exp(10*(t-6))) #at t=8 changes to 0.3
        if(t<7):
            alpha = 0.5
        else:
            alpha = 0.3
        # alpha = self.alpha_0 / (1 + self.beta * t) + self.alpha_stab    # inverse time decay
        # alpha = self.alpha_0 * np.exp(-self.beta * t) + self.alpha_stab   # exponential (stabilizes in alpha=0.3)
        print("Alpha: ", alpha)
        return alpha

    def huber_psi(self, r):
        """Huber influence function"""
        huber = np.where(np.abs(r) <= self.c, r, self.c * np.sign(r)) #sign indicates wether to increment or decrease cost
        print("Huber: ", huber)
        return huber

    def update_cost(self, i, j, observed_cost, n_exp):
        """Update cost entry (i, j) using Huber M-estimator"""
        print("Previous cost: ", self.cost_table[i, j])
        print("Observation: ", observed_cost)

        residual = observed_cost - self.cost_table[i, j]
        print("Residual: ", residual)

        self.huber = self.huber_psi(residual)

        self.alpha = self.get_alpha(n_exp)
        # self.cost_table[i, j] += self.alpha * self.huber_psi(residual)
        # print("Cost update: ", self.alpha * self.huber_psi(residual))
        self.cost_table[i, j] += self.alpha * self.huber
        print("Cost update: ", self.alpha * self.huber)

    def get_cost_table(self):
        """Return the updated cost table"""
        return self.cost_table

    def save_cell_evolution(self, matrix):
        for i in range(3):
            for j in range(3):
                self.cells[i][j].append(matrix[i, j]) ## Save costs in separated arrays to be plotted

        return self.cells
        
    def plot_costs(self, data):

        names = [['A vertical', 'A diagonal', 'A rotating'], ['B vertical', 'B diagonal', 'B rotating'], ['C vertical', 'C diagonal', 'C rotating']]
        colors = [['black', 'black', 'black'], ['cyan', 'green', 'pink'], ['blue', 'yellow', 'red']]

        # n_trials = len(data[0][0])
        x = np.arange(0, len(data[0][0]))

        # Create a smooth parameterized curve using cubic splines
        # t = np.linspace(0, 1, len(x))  # Normalized parameter
        # print(t)
        # cs_x = CubicSpline(t, x)  # X interpolation
        # # Generate fine-grained trajectory points
        # t_fine = np.linspace(0, 1, 100)
        # x_smooth = cs_x(t_fine)

        fig = plt.figure(figsize=(8, 6))

        ## Plot place cost table updates
        for i in range(3): #rows
            for j in range(3): #columns 
                name = names[i][j]
                color = colors[i][j]
                plt.plot(x, data[i][j], 'bo', linestyle='-', linewidth=2, color=color, label=name)  # Waypoints as red dots
                # cs_y = CubicSpline(t, data[i][j]) 
                # y_smooth = cs_y(t_fine)
                # plt.plot(x, placed_quality_results, 'ro')  # Waypoints as red dots
                # plt.plot(x_smooth, y_smooth, 'g-', label="A vertical")  # Smooth curve
                # plt.plot(range(10), data[i][j], label=f"Cell ({i},{j})")

        ## Plot config
        plt.xticks(x)
        plt.xlabel("Experiment")
        plt.ylabel("Current cost (placing error)")
        plt.legend()
        plt.grid()

        return fig

    def plot_quality(self, placed_quality_results, piled_quality_results, labels):
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
        fig = plt.figure(figsize=(8, 6))
        plt.plot(x, placed_quality_results, 'ro')  # Waypoints as red dots
        plt.plot(x_smooth, yplaced_smooth, 'g-', label="Placed quality")  # Smooth curve
        plt.plot(x, piled_quality_results, 'ro')  # Waypoints as red dots
        plt.plot(x_smooth, ypiled_smooth, 'b-', label="Pile quality")  # Smooth curve

        #Put labels to points
        for i,j in zip(x,placed_quality_results):
            plt.annotate(labels[i], (i+0.05,j+0.05))

        plt.xticks(x)
        plt.xlabel("Experiment")
        plt.ylabel("Placing quality")
        plt.title("Placing quality through experiments")
        plt.legend()
        plt.grid()
        # plt.show()

        return fig

####################################################################################


## ---- Placing cost table ----
# Def clas | Vertical | Diagonal | Rotating
#    A     |    0     |    0     |    0
#    B     |    0     |    0     |    0
#    C     |    0     |    0     |    0
place_initial_cost_table = np.array([ # Init cost table (placing error to minimize)
    [0, 0, 0],
    [0, 0, 0], 
    [0, 0, 0], 
])  
pile_initial_cost_table = np.array([ # Init cost table (placing error to minimize)
    [0, 0, 0],
    [0, 0, 0], 
    [0, 0, 0], 
])  
cost_tables = [place_initial_cost_table, pile_initial_cost_table]
costs_history = [[[[] for _ in range(3)] for _ in range(3)], [[[] for _ in range(3)] for _ in range(3)]]

# costs_history[0] = updater.save_cell_evolution(place_initial_cost_table)
# costs_history[1] = updater.save_cell_evolution(pile_initial_cost_table)
# updater = CostUpdater(place_initial_cost_table, 0.5, 0.3, 45, 0.5)

# n_exp = 1

for m in range(0,len(errors)):
    updater = CostUpdater(cost_tables[m], 0.5, 0.3, 60, 0.5)
    costs_history[m] = updater.save_cell_evolution(cost_tables[m]) #Initialize cells
    for n in range(1,len(placed_quality_results)): 
        def_class = classes[m][n]
        placing_str = strategies[m][n]
        if(def_class=="A"):
            i=0
        elif(def_class=="B"):
            i=1
        elif(def_class=="C"):
            i=2
        if(placing_str=="v"):
            j=0
        elif(placing_str=="d"):
            j=1
        elif(placing_str=="r"):
            j=2
        # updater.get_alpha(n_exp)
        updater.update_cost(i,j, errors[m][n], n)
        costs_matrix = updater.get_cost_table()
        print("Updated Cost Table:\n", costs_matrix)

        costs_history[m] = updater.save_cell_evolution(costs_matrix)
        print(costs_history[m])

        print("--------------------")
    print("---------------------------------------------")
    print("---------------------------------------------")


updater.plot_quality(placed_quality_results, piled_quality_results, labels)

updater.plot_costs(costs_history[0])
plt.title("PLACE Cost update evolution")
updater.plot_costs(costs_history[1])
plt.title("PILE Cost update evolution")
# ax.legend(title='Cost cell') # Def class and placing action combination cost
plt.show()


