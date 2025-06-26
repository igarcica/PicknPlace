
### SYSTEM SCALABILITY RESULTS
### This script plots the accumulated quality of the piled objects 

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import cost_update as cost_update


## init cost table is previous learned one
place_initial_cost_table = np.array([ # Init cost table (placing error to minimize)
    [17, 1, 1],
    [8, 22, 6], 
    [30, 25, 6], 
])  
pile_initial_cost_table = np.array([ # Init cost table (placing error to minimize)
    [7, 3, 4],
    [30, 14, 9], 
    [30, 30, 30], 
])  

################## SYSTEM'S SCALABILITY ##################
######### TOWEL 8l - pile of 4 objects ######### - 
quality_results = np.array([99, 94, 91, 78]) #1.1 to 1.4
accumulated_quality = np.cumsum(quality_results)
indices = np.arange(1, len(accumulated_quality) + 1) # Accumulated error of pile
norm_accum_quality = accumulated_quality / indices # percentage (%)

placing_errors = 100-norm_accum_quality
placing_str = ["d", "d", "d", "d"]
placing_def_classes = ["A", "A", "A", "A"]

##################
# placed_object = 0
piled_objects = len(quality_results)
# print(placed_object)
# print(piled_objects)

errors = [[placing_errors[0]], placing_errors[1:piled_objects]]
print(errors)
strategies = [placing_str[0], placing_str[1:piled_objects]]
classes = [placing_def_classes[0], placing_def_classes[1:piled_objects]]

delta=45

cost_tables = [place_initial_cost_table, pile_initial_cost_table]
costs_history = [[[[] for _ in range(3)] for _ in range(3)], [[[] for _ in range(3)] for _ in range(3)]] #cells evolution (matrix of vector of each cell)
cost_table_evolution = [[],[]] #matrix of matrices corresponding to the cost table at each timestep
changes=[[],[]] #values which change (to plot the circles)
changes[0].append(0) #cloth-to-table
changes[1].append(0) #cloth-to-cloth


def plot_quality(quality_results):
        # Create a smooth parameterized curve using cubic splines
        x = np.arange(0, len(quality_results)) #np.linspace(0,1, len(x))
        t = np.linspace(0, 1, len(x))  # Normalized parameter
        cs_x = CubicSpline(t, x)  # X interpolation
        cs_ypiled = CubicSpline(t, quality_results)  # Y interpolation

        # Generate fine-grained trajectory points
        t_fine = np.linspace(0, 1, 100)
        x_smooth = cs_x(t_fine)
        ypiled_smooth = cs_ypiled(t_fine)

        # Plot the trajectory
        fig = plt.figure(figsize=(9, 6))

        plt.scatter(x, quality_results, color='black', zorder=3) 
        plt.plot(x_smooth, ypiled_smooth, color="royalblue", linewidth=2.5, linestyle="-", alpha=0.8) #"#FF5733 "#33CFFF"

        # #Put labels to points
        # for i,j in zip(x,placed_quality_results):
        #     plt.annotate(labels[i], (i+0.05,j+0.05))

        plt.ylim(0,105)
        plt.xticks(x)
        plt.xlabel("Trial", fontsize=18)
        plt.ylabel("Pile quality (%)", fontsize=18)
        plt.title("Pile quality evolution over trials", fontsize=20, fontweight='bold', color="#333333")
        plt.grid(True,  linewidth=0.8, alpha=0.5)
        plt.gca().spines["top"].set_visible(False)
        plt.gca().spines["right"].set_visible(False)
        # plt.show()

        return fig


####################################################################################
##Plot quality of each object in the pile
plot_quality(norm_accum_quality)
plt.show()


## Update cost after placing each object in the pile
for m in range(0,len(errors)):  #Update the two tables
    print("m", m)
    updater = cost_update.CostUpdater(cost_tables[m], 0.5, 0.3, 60, 0.5)
    costs_history[m] = updater.save_cell_evolution(cost_tables[m]) #Initialize cells
    cost_table_evolution[m].append(cost_tables[m].copy()) #Initialize cost tables history
    for n in range(0,len(errors[m])): # Consider all objects (first placed object and following piled objects)
        print("n", n)
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
        hola = updater.update_cost(i,j, errors[m][n], n)
        costs_matrix = updater.get_cost_table()
        print("Updated Cost Table:\n", costs_matrix)

        costs_history[m] = updater.save_cell_evolution(costs_matrix)
        print(costs_history[m])
        changes[m].append(hola)
        print(costs_matrix)
        cost_table_evolution[m].append(costs_matrix.copy())

        
        print(type(cost_table_evolution))

        print("--------------------")
    print("---------------------------------------------")
    # print("Last cost table: \n", costs_matrix)
    print("---------------------------------------------")

