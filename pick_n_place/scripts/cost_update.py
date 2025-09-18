# ### SYSTEM ADAPTABILITY RESULTS
# ### This script collects the placing_costs_update.py and the plot_placing_quality.py

import numpy as np
# import statistics as sts
import matplotlib.pyplot as plt
from cycler import cycler
from scipy.interpolate import CubicSpline
from scipy.interpolate import make_interp_spline
from scipy.interpolate import PchipInterpolator
# from matplotlib.animation import FuncAnimation
# from scipy.interpolate import make_interp_spline

## UTIL FUNCTIONS
activate_print=True

def print_info(activate, arg1, arg2="", arg3="", arg4="", arg5="", arg6=""):
    if(activate):
        print(str(arg1) + str(arg2) + str(arg3) + str(arg4) + str(arg5) + str(arg6))

# ####################################################################################
# ### INPUT DATA

# ## ---- Placing cost table ----
# # Def clas | Vertical | Diagonal | Rotating
# #    A     |    0     |    0     |    0
# #    B     |    0     |    0     |    0
# #    C     |    0     |    0     |    0
# # place_initial_cost_table = np.array([ # Init cost table (placing error to minimize)
# #     [17, 0, 0],
# #     [8, 22, 6], 
# #     [30, 25, 6], 
# # ])  
# # pile_initial_cost_table = np.array([ # Init cost table (placing error to minimize)
# #     [0, 0, 0],
# #     [30, 14, 9], 
# #     [30, 30, 30], 
# # ]) 
# place_initial_cost_table = np.array([ # Init cost table (placing error to minimize)
#     [0, 0, 0],
#     [0, 0, 0], 
#     [0, 0, 0], 
# ])  
# pile_initial_cost_table = np.array([ # Init cost table (placing error to minimize)
#     [0, 0, 0],
#     [0, 0, 0], 
#     [0, 0, 0], 
# ])  

# ####################################################################################

# #### PILLOWCASE ####
# # placed_quality_results = np.array([0, 13, 49, 88, 94, 65, 56, 94, 97, 91, 90, 95, 96, 83, 86, 89, 96,94,  88,  94, 92]) #exp31(t=10) bit changed
# # placing_errors = 100-placed_quality_results
# # placing_str = ["0", "v", "d", "r", "v", "v", "d", "r", "r", "r", "v", "r", "r", "r", "v", "v", "r", "r", "r", "r", "r"]
# # placing_def_classes = ["0", "C", "C", "C", "B", "A", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B"]

# # piled_quality_results = np.array([0, 0, 0, 8, 31, 81, 86, 78, 78, 93, 78, 79, 85, 93, 77, 88, 93,91, 85,  92, 91]) #exp31(t=10) bit changed
# # piling_errors = 100-piled_quality_results
# # piling_str = ["0", "v", "d", "r", "v", "d", "r", "r", "d", "r", "r", "d", "r", "r", "r", "r", "r", "r", "r", "r", "r"] 
# # piling_def_classes = ["0", "C", "C", "C", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B"]

# # labels = np.array(["0", "vv", "dd", "rr", "vv", "vd", "dr", "rr", "rd", "rr", "vr", "rd", "rr", "rr", "vr", "vr", "rr", "rr", "rr", "rr", "rr"])

# # print(sts.mean([0,0,8]))
# # print(sts.mean([77, 88, 91]))

# ######### TOWEL #########
# # placed_quality_results = np.array([0, 92, 92, 94, 96]) 
# # placing_errors = 100-placed_quality_results
# # placing_str = ["0", "v", "d", "r", "r"]
# # placing_def_classes = ["0", "A", "A", "A", "A"]
# # piled_quality_results = np.array([0, 70, 89, 80, 91]) 
# # piling_errors = 100-piled_quality_results
# # piling_str = ["0", "v", "d", "r", "d"] 
# # piling_def_classes = ["0", "A", "A", "A", "A"]

# # labels = np.array(["0", "vv", "dd", "rr", "rd"])

# ##Starting with learned cost table from pillowcase
# # placed_quality_results = np.array([0, 98, 99, 96, 99, 99, 98, 99, 98]) #95, 97, 96, 96, 96]) 
# # placing_errors = 100-placed_quality_results
# # placing_str = ["0", "d", "r", "r", "d", "d", "d", "d", "d"]
# # placing_def_classes = ["0", "A", "A", "A", "A", "A", "A", "A", "A"]
# # piled_quality_results = np.array([0, 76, 88, 91, 90, 94, 95, 86, 90])#29, 68, 73, 70, 91]) 
# # piling_errors = 100-piled_quality_results
# # piling_def_classes = ["0", "A", "A", "A", "A", "A", "A", "A", "A"]
# # piling_str = ["0", "v", "d", "r", "r", "d", "d", "d", "d"] 

# # labels = np.array(["0", "dv", "rd", "rr", "dr", "rd", "dd", "dd", "dd", "dd"])

# ######### PILLOWCASE ONLY LONG EDGE #########
# # placed_quality_results = np.array([0, 13, 49, 88, 91, 83, 91,88, 78, 92]) 
# # placing_errors = 100-placed_quality_results
# # placing_str = ["0", "v", "d", "r", "r", "r", "r", "r", "r", "r"]
# # placing_def_classes = ["0", "C", "C", "C", "C", "C", "C", "C", "C", "C"]
# # piled_quality_results = np.array([0, 0,0,8,42,43,59, 0, 22, 33]) 
# # piling_errors = 100-piled_quality_results
# # piling_str = ["0", "v", "d", "r", "v", "d", "r", "r", "d", "v"] 
# # piling_def_classes = ["0", "C", "C", "C", "C", "C", "C", "C", "C", "C"]
# # labels = np.array(["0", "vv", "dd", "rr", "rv", "rd", "rr", "rr", "rd", "rv"])


# ################## SYSTEM'S ADAPTABILITY ##################
# ######### PILLOWCASE + TOWEL #########
# # placed_quality_results = np.array([0, 13, 49, 88, 94, 65, 56, 94, 97, 91, 90, 95, 96, 83, 86, 89, 96, 94, 88, 94, 92, 98, 99, 96, 99, 99, 98]) #exp31(t=10) bit changed
# # placing_errors = 100-placed_quality_results
# # placing_str = ["0", "v", "d", "r", "v", "v", "d", "r", "r", "r", "v", "r", "r", "r", "v", "v", "r", "r", "r", "r", "r", "d", "r", "r", "d", "d", "d", "d"]
# # placing_def_classes = ["0", "C", "C", "C", "B", "A", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "A", "A", "A", "A", "A", "A", "A"]

# # piled_quality_results = np.array([0, 0, 0, 8, 31, 81, 86, 78, 78, 93, 78, 79, 85, 93, 77, 88, 93, 91, 85, 92, 91, 76, 88, 91, 90, 94, 95]) #exp31(t=10) bit changed
# # piling_errors = 100-piled_quality_results
# # piling_str = ["0", "v", "d", "r", "v", "d", "r", "r", "d", "r", "r", "d", "r", "r", "r", "r", "r", "r", "r", "r", "r", "v", "d", "r", "r", "d", "d", "d"] 
# # piling_def_classes = ["0", "C", "C", "C", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "A", "A", "A", "A", "A", "A", "A"]

# # labels = np.array(["0", "vv", "dd", "rr", "vv", "vd", "dr", "rr", "rd", "rr", "vr", "rd", "rr", "rr", "vr", "vr", "rr", "rr", "rr", "rr", "rr", "dv", "rd", "rr", "dr", "rd", "dd", "dd", "dd"])

# ################## SYSTEM'S ADAPTABILITY ################## notion exps values
# # ######### PILLOWCASE + TOWEL #########
# placed_quality_results = np.array([0, 13, 49, 88, 94, 65, 56, 94, 97, 91, 90, 95, 96, 83, 86, 89, 96, 94, 88, 94, 92, 96, 99, 96, 99, 99, 98]) #trials 21 y 23 lowered wrt notion values (from 98 to 96)
# placing_errors = 100-placed_quality_results
# placing_str = ["0", "v", "d", "r", "v", "v", "d", "r", "r", "r", "v", "r", "r", "r", "v", "v", "r", "r", "r", "r", "r", "d", "r", "r", "d", "d", "d"]
# placing_def_classes = ["0", "C", "C", "C", "B", "A", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "A", "A", "A", "A", "A", "A"]

# piled_quality_results = np.array([0, 0, 0, 8, 31, 81, 86, 78, 78, 93, 78, 79, 85, 93, 77, 88, 93, 91, 85, 92, 91, 76, 88, 91, 90, 94, 95]) 
# piling_errors = 100-piled_quality_results
# piling_str = ["0", "v", "d", "r", "v", "d", "r", "r", "d", "r", "r", "d", "r", "r", "r", "r", "r", "r", "r", "r", "r", "v", "d", "r", "r", "d", "d"] 
# piling_def_classes = ["0", "C", "C", "C", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "A", "A", "A", "A", "A", "A"]

# labels = np.array(["0", "vv", "dd", "rr", "vv", "vd", "dr", "rr", "rd", "rr", "vr", "rd", "rr", "rr", "vr", "vr", "rr", "rr", "rr", "rr", "rr", "dv", "rd", "rr", "dr", "dd", "dd"])



# ################## SYSTEM'S PERFORMANCE ##################
# ######### TOWEL 8l ######### - init cost table is previous learned one
# # place_initial_cost_table = np.array([ # Init cost table (placing error to minimize)
# #     [17, 1, 1],
# #     [8, 22, 6], 
# #     [30, 25, 6], 
# # ])  
# # pile_initial_cost_table = np.array([ # Init cost table (placing error to minimize)
# #     [7, 3, 4],
# #     [30, 14, 9], 
# #     [30, 30, 30], 
# # ])  

# # #exps: 4.1 to 6.2 of system performance
# # placed_quality_results = np.array([100, 100, 99, 99]) 
# # placing_errors = 100-placed_quality_results
# # placing_str = ["0", "d", "d", "d"]
# # placing_def_classes = ["0", "A", "A", "A"]

# # piled_quality_results = np.array([95, 95, 99,  99]) 
# # piling_errors = 100-piled_quality_results
# # piling_str = ["0", "d", "d", "d"] 
# # piling_def_classes = ["0", "A", "A", "A"]

# # labels = np.array(["0", "dd", "dd", "dd"])


# ##########################################
# errors = [placing_errors, piling_errors]
# strategies = [placing_str, piling_str]
# classes = [placing_def_classes, piling_def_classes]



# delta=45
# # print("Std deviation: ", sts.stdev(placing_errors))
# # print("Std deviation: ", sts.stdev(piling_errors))


####################################################################################
class CostUpdater:
    def __init__(self, cost_table, alpha_0=1, alpha_stab=0.3, delta=30, beta=1):
        """
        Initialize with:
        - cost_table: initial cost matrix (numpy array)
        - alpha: learning rate for updates (eta (n) in the paper)
        - delta: threshold for Huber loss
        """
        self.cost_table = cost_table
        self.alpha_0 = alpha_0
        self.alpha_stab = alpha_stab
        self.delta = delta
        self.beta = beta                #beta 1.5 stabilize alpha at step 6
        self.cells = [[[] for _ in range(3)] for _ in range(3)] # Create a storage list for each cell

    def get_alpha(self, t):
        """Compute decayed alpha based on the selected decay type."""
        # self.alpha = self.alpha_0 # Fized alpha
        # alpha = self.alpha_0 + (1-t)*t #en pruebas
        # alpha = self.alpha_stab + (self.alpha_0-self.alpha_stab)/(1+np.exp(10*(t-6))) #at t=8 changes to 0.3
        if(t<7):
            alpha = 0.3 #0.5
        else:
            alpha = 0.3
        # alpha = self.alpha_0 / (1 + self.beta * t) + self.alpha_stab    # inverse time decay
        # alpha = self.alpha_0 * np.exp(-self.beta * t) + self.alpha_stab   # exponential (stabilizes in alpha=0.3)
        print_info(activate_print,"Alpha: ", alpha)
        return alpha

    def huber_psi(self, r):
        """Huber influence function"""
        huber = np.where(np.abs(r) <= self.delta, r, self.delta * np.sign(r)) #sign indicates wether to increment or decrease cost
        print_info(activate_print,"Huber: ", huber)
        return huber

    def update_cost(self, i, j, observed_error, n_exp):
        """Update cost entry (i, j) using Huber M-estimator"""
        print_info(activate_print,"Previous cost: ", self.cost_table[i, j])
        print_info(activate_print,"Observation: ", observed_error)

        residual = observed_error - self.cost_table[i, j] #et(dt, pt) - Ct(dt, pt)
        print_info(activate_print,"Residual: ", residual)
        print_info(activate_print,"Previous Cost update: ", self.cost_table[i, j])

        self.huber = self.huber_psi(residual)
        self.alpha = self.get_alpha(n_exp)

        # self.cost_table[i, j] += self.alpha * self.huber_psi(residual)
        # print_info(activate_print,"Cost update: ", self.alpha * self.huber_psi(residual))
        # self.cost_table[i, j] += self.alpha * self.huber
        # print_info(activate_print,"Cost update: ", self.alpha * self.huber)
        result = self.alpha * self.huber
        self.cost_table[i, j] += round(result)
        # self.cost_table[i, j] += result
        print_info(activate_print,"Cost update: ", result)
        print_info(activate_print,"Rounded Cost update: ", round(result))
        print_info(activate_print,"Rounded Cost update: ", self.cost_table[i, j])

        return self.cost_table[i, j]

    def get_cost_table(self):
        """Return the updated cost table"""
        return self.cost_table

    def save_cell_evolution(self, matrix):
        for i in range(3):
            for j in range(3):
                self.cells[i][j].append(matrix[i, j]) ## Save costs in separated arrays to be plotted

        return self.cells
        
    def plot_costs(self, axes, data, points):

        names = [['A vertical', 'A diagonal', 'A rotating'], ['B vertical', 'B diagonal', 'B rotating'], ['C vertical', 'C diagonal', 'C rotating']]
        colors = [['black', 'black', 'black'], ['cyan', 'green', 'pink'], ['blue', 'yellow', 'red']]
        plt.rc('axes', prop_cycle=cycler(color=plt.cm.hsv(np.linspace(0, 1, 9))))  # gama de colores automatica para las lineas

        # n_trials = len(data[0][0])
        x = np.arange(0, len(data[0][0])) #number of trials

        # Create a smooth parameterized curve using cubic splines
        # t = np.linspace(0, 1, len(x))  # Normalized parameter
        # print(t)
        # cs_x = CubicSpline(t, x)  # X interpolation
        # # Generate fine-grained trajectory points
        # t_fine = np.linspace(0, 1, 100)
        # x_smooth = cs_x(t_fine)

        ## Plot place cost table updates
        for i in range(3): #rows
            for j in range(3): #columns 
                name = names[i][j]
                color = colors[i][j]
                axes.plot(x, data[i][j], linestyle='-', linewidth=2, label=name)  # Waypoints as red dots
                # plt.plot(x, data[i][j], 'bo', linestyle='-', linewidth=2, color=color, label=name)  # Waypoints as red dots
                # cs_y = CubicSpline(t, data[i][j]) 
                # y_smooth = cs_y(t_fine)
                # plt.plot(x, placed_quality_results, 'ro')  # Waypoints as red dots
                # plt.plot(x_smooth, y_smooth, 'g-', label="A vertical")  # Smooth curve
                # plt.plot(range(10), data[i][j], label=f"Cell ({i},{j})")

        axes.scatter(x, points, color='black', zorder=3)

        # axes.axvline(x=20, color='gray', linestyle='--', linewidth=2) #Plot a dashed line when plotting the pillowcase + towel results

        ## Plot config
        axes.set_xticks(x)
        axes.set_xlabel("Trial", fontsize=18)
        # axes.set_ylabel("Current cost (placing error)")
        # axes.legend()
        # plt.legend(fontsize=12, loc="upper right", frameon=True, fancybox=True, shadow=True, borderpad=1) # Add a legend with a nice style
        axes.grid()
        axes.grid(True,  linewidth=0.8, alpha=0.5)
        

        # return fig

    def plot_quality(self, placed_quality_results, piled_quality_results, labels):
        # # Create a smooth parameterized curve using cubic splines
        # x = np.arange(0, len(placed_quality_results)) #np.linspace(0,1, len(x))
        # t = np.linspace(0, 1, len(x))  # Normalized parameter
        # cs_x = CubicSpline(t, x)  # X interpolation
        # cs_yplaced = CubicSpline(t, placed_quality_results)  # Y interpolation
        # cs_ypiled = CubicSpline(t, piled_quality_results)  # Y interpolation

        # # Generate fine-grained trajectory points
        # t_fine = np.linspace(0, 1, 50)
        # x_smooth = cs_x(t_fine)
        # yplaced_smooth = cs_yplaced(t_fine)
        # ypiled_smooth = cs_ypiled(t_fine)

        # # cs_yplaced = make_interp_spline(t, piled_quality_results, k=1)
        # # ypiled_smooth = cs_yplaced(t_fine)

        # # Plot the trajectory
        # fig = plt.figure(figsize=(9, 6))
        # # plt.plot(x, placed_quality_results, 'bo')  # Waypoints as red dots
        # plt.scatter(x, placed_quality_results, color='black', zorder=3) 
        # plt.plot(x_smooth, yplaced_smooth, 'g-', label="Placing quality", linewidth=2.5, linestyle="-", alpha=0.8)  # Smooth curve
        # # # plt.plot(x, piled_quality_results, 'ro')  # Waypoints as red dots
        # # # plt.plot(x_smooth, ypiled_smooth, 'b-', label="Pile quality")  # Smooth curve
        # # plt.scatter(x, piled_quality_results, color='black')  
        # # plt.plot(x_smooth, ypiled_smooth, 'b-')  # Smooth curve

        # plt.scatter(x, piled_quality_results, color='black', zorder=3) 
        # plt.plot(x_smooth, ypiled_smooth, color="royalblue", label="Piling quality", linewidth=2.5, linestyle="-", alpha=0.8) #"#FF5733 "#33CFFF"
        # plt.plot(x_smooth, piled_quality_results, color="royalblue", label="Piling quality", linewidth=2.5, linestyle="-", alpha=0.8) #"#FF5733 "#33CFFF"



        x = np.arange(0, len(placed_quality_results))
        t = np.linspace(0, 1, len(x))

        # Use PCHIP instead of cubic spline
        pchip_yplaced = PchipInterpolator(t, placed_quality_results)
        pchip_ypiled = PchipInterpolator(t, piled_quality_results)

        t_fine = np.linspace(0, 1, 200)
        x_smooth = np.interp(t_fine, t, x)  # linear interp for x
        yplaced_smooth = pchip_yplaced(t_fine)
        ypiled_smooth = pchip_ypiled(t_fine)

        fig = plt.figure(figsize=(11, 8))
        # fig, ax = plt.subplots(figsize=(10, 9))  # create fig + axis
        plt.scatter(x, placed_quality_results, color='black', zorder=3)
        plt.plot(x_smooth, yplaced_smooth, 'g-', label="Placing quality", linewidth=3, alpha=0.8)
        plt.scatter(x, piled_quality_results, color='black', zorder=3)
        plt.plot(x_smooth, ypiled_smooth, color="royalblue", linestyle="-", label="Piling quality", linewidth=3, alpha=0.8)

        # #Put labels to points
        # for i,j in zip(x,placed_quality_results):
        #     plt.annotate(labels[i], (i+0.05,j+0.05))

        # plt.axvline(x=20, color='gray', linestyle='--', linewidth=2) #Plot a dashed line when plotting the pillowcase + towel results

        plt.xticks(x)
        print(np.linspace(0,100,10))
        plt.yticks(np.linspace(0,100,11))
        # plt.xlabel("Trial (Complete pile)", fontsize=18)
        plt.ylabel("Quality (%)", fontsize=18)
        plt.title("Quality evolution over trials", fontsize=20, fontweight='bold', color="#333333")
        plt.legend(loc="center right", fontsize=12) 
        # plt.grid()
        # Customize the grid and spines
        plt.grid(True,  linewidth=0.8, alpha=0.5)
        plt.gca().spines["top"].set_visible(False)
        plt.gca().spines["right"].set_visible(False)
        # Extra labels (smaller font) placed above
        # ax.set_xlabel("Trial (Complete pile)", fontsize=18)
        # ax.text(0.5, -0.02, "Piles of Checkered rags", ha="center", va="center", transform=ax.transAxes, fontsize=12)
        # ax.text(0.5, -0.18, "Piles of Waffle rags", ha="center", va="center", transform=ax.transAxes, fontsize=9)
        plt.xlabel("Trial (Complete pile)", labelpad=25, fontsize=18)  # increase spacing
        plt.text(0.35, -0.07, "Piles of Checkered rags", ha="center", transform=plt.gca().transAxes, fontsize=12)
        plt.text(0.83, -0.07, "Piles of Waffle rags", ha="center", transform=plt.gca().transAxes, fontsize=12)

        # plt.show()

        return fig

####################################################################################



# cost_tables = [place_initial_cost_table, pile_initial_cost_table]
# costs_history = [[[[] for _ in range(3)] for _ in range(3)], [[[] for _ in range(3)] for _ in range(3)]] #cells evolution (matrix of vector of each cell)
# cost_table_evolution = [[],[]] #matrix of matrices corresponding to the cost table at each timestep
# #
# changes=[[],[]] #values which change (to plot the circles)
# # #Start with table costs
# # for m in range(2): #placing and piling costs
# #     for i in range(3):
# #         for j in range(3):
# #             changes[m][i][j].append(matrix[i, j]) ## Save costs in separated arrays to be plotted
# changes[0].append(0) #cloth-to-table
# changes[1].append(0) #cloth-to-cloth
# # costs_history[0] = updater.save_cell_evolution(place_initial_cost_table)
# # costs_history[1] = updater.save_cell_evolution(pile_initial_cost_table)
# # updater = CostUpdater(place_initial_cost_table, 0.5, 0.3, 45, 0.5)

# # cost_table_evolution #matrix of matrices corresponding to the cost table at each timestep

# # n_exp = 1

# for m in range(0,len(errors)):
#     updater = CostUpdater(cost_tables[m], 0.5, 0.3, 60, 0.5)
#     costs_history[m] = updater.save_cell_evolution(cost_tables[m]) #Initialize cells
#     cost_table_evolution[m].append(cost_tables[m].copy()) #Initialize cost tables history
#     for n in range(1,len(placed_quality_results)): 
#         def_class = classes[m][n]
#         placing_str = strategies[m][n]
#         if(def_class=="A"):
#             i=0
#         elif(def_class=="B"):
#             i=1
#         elif(def_class=="C"):
#             i=2
#         if(placing_str=="v"):
#             j=0
#         elif(placing_str=="d"):
#             j=1
#         elif(placing_str=="r"):
#             j=2
#         # updater.get_alpha(n_exp)
#         hola = updater.update_cost(i,j, errors[m][n], n)
#         costs_matrix = updater.get_cost_table()
#         print_info(True,"Updated Cost Table:\n", costs_matrix)

#         costs_history[m] = updater.save_cell_evolution(costs_matrix)
#         print_info(activate_print,costs_history[m])
#         changes[m].append(hola)
#         print(costs_matrix)
#         cost_table_evolution[m].append(costs_matrix.copy())

        
#         print(type(cost_table_evolution))

#         print_info(activate_print,"--------------------")
#     print("---------------------------------------------")
#     print("Last cost table: \n", costs_matrix)
#     print("---------------------------------------------")

# ############################

# # print("Final PLACING cost table: ")

# # print("Final PILING cost table: ")
# # print(costs_history[1])
# updater.plot_quality(placed_quality_results, piled_quality_results, labels)

# fig, axes = plt.subplots(nrows=2, ncols=1, figsize=(9, 8))  # Create a 2-row, 3-column figure
# # fig = plt.figure(figsize=(8, 6))
# updater.plot_costs(axes[0], costs_history[0], changes[0])
# axes[0].set_ylabel("cloth-to-table cost", fontsize=18)
# # plt.title("PLACE Cost update evolution")

# updater.plot_costs(axes[1], costs_history[1], changes[1])
# axes[1].set_ylabel("cloth-to-cloth cost", fontsize=18)
# # plt.title("PILE Cost update evolution")

# fig.suptitle("Cost update over trials", fontsize=20, fontweight='bold', color="#333333")
# handles, labels = plt.gca().get_legend_handles_labels()  # Get all lines
# fig.legend(handles[:9], labels[:9], title='State-Action Cost')  # # Def class and placing action combination cost - Show only the first 9
# plt.show()


# ##########################################################################################
# ###### ANIMATED PLOTS
# ### Placing quality evolution

# x = np.linspace(0, 27, len(piled_quality_results))

# # Interpolation
# x_smooth = np.linspace(x.min(), x.max(), 500)
# spline = make_interp_spline(x, piled_quality_results, k=3)
# y_smooth = spline(x_smooth)

# # Setup
# fig, ax = plt.subplots(figsize=(10, 6))
# line, = ax.plot([], [], lw=2, color='#1f77b4')
# point, = ax.plot([], [], 'ro')
# scatter = ax.scatter([], [], color='black', s=50, zorder=3)  # Hollow circles

# ax.set_xlim(x.min(), x.max())
# ax.set_ylim(piled_quality_results.min() - 10, piled_quality_results.max() + 10)

# plt.axvline(x=x[20], color='gray', linestyle='--', linewidth=2) #Plot a dashed line when plotting the pillowcase + towel results

# # Styling
# ax.set_xlabel("Trial", fontsize=18)
# ax.set_ylabel("Pile quality (%)", fontsize=18)
# ax.set_title("Pile quality evolution over trials", fontsize=20, fontweight='bold', color="#333333")
# ax.grid(True, linewidth=0.8, alpha=0.5)
# ax.spines["top"].set_visible(False)
# ax.spines["right"].set_visible(False)

# # Example custom labels (must match length of x)
# custom_labels = [f"{i}" for i in range(len(x))]  # e.g., T0, T1, ..., T26
# ax.set_xticks(x) # Set ticks at original data point positions
# ax.set_xticklabels(custom_labels, fontsize=10)  # Rotate for readability


# # Animation
# def init_plot_quality_anim():
#     line.set_data([], [])
#     point.set_data([], [])
#     scatter.set_offsets([])
#     return line, point, scatter

# def update_plot_quality_anim(frame):
#     # Curve line + red dot
#     line.set_data(x_smooth[:frame], y_smooth[:frame])
#     point.set_data(x_smooth[frame - 1], y_smooth[frame - 1])

#     # Show markers up to the current x_smooth point
#     current_x = x_smooth[frame - 1]
#     visible_indices = np.where(x <= current_x)[0]
#     scatter.set_offsets(np.column_stack((x[visible_indices], piled_quality_results[visible_indices])))

#     return line, point, scatter

# # Animate
# ani = FuncAnimation(fig, update_plot_quality_anim, frames=len(x_smooth), init_func=init_plot_quality_anim,
#                     blit=False, interval=15, repeat=False)

# plt.tight_layout()

# # Save as video
# ani.save("pile_quality_animation.mp4", writer="ffmpeg", fps=30)
# plt.show()


# ######## Cost table evolution 

# matrices=cost_table_evolution[0]
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

# # ani.save("cost_update.mp4", writer="ffmpeg", fps=30)
# plt.show()