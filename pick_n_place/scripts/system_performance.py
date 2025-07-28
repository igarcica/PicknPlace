
### SYSTEM PERFORMANCE RESULTS
### This script plots the place and pile quality of the system's performance experiments (piles of 2 objects)
### Objects: waffle rag 8l, chekered rag 6l (folded in thirds + halves)

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import cost_update as cost_update
from cycler import cycler

activate_print=True

## init cost table is the previous learned one from system's adaptability experiments
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

################## SYSTEM'S PERFORMANCE ##################
# ######### WAFFLE RAG 8L #########
placed_quality_results = np.array([0, 99, 94, 93, 92]) #trials 20 to 23
placing_errors = 100-placed_quality_results
placing_str = ["0", "d", "d", "r", "r"]
placing_def_classes = ["0", "A", "A", "B", "B"]

piled_quality_results = np.array([0, 81, 86, 96, 95]) #trials 20 to 23
piling_errors = 100-piled_quality_results
piling_str = ["0", "r", "r", "d", "r"] 
piling_def_classes = ["0", "B", "B", "A", "B"]

labels = np.array(["0", "dr"])

# # ######### CHECKERED RAG 6L #########
# placed_quality_results = np.array([0, 96]) #trials 26
# placing_errors = 100-placed_quality_results
# placing_str = ["0", "r"]
# placing_def_classes = ["0", "B"]

# piled_quality_results = np.array([0, 95]) #trials 26
# piling_errors = 100-piled_quality_results
# piling_str = ["0", "r"] 
# piling_def_classes = ["0", "B"]

# labels = np.array(["0", "rr"])


######### ALL - TOWEL 8L, WAFFLE 8L, CHEKERED 6L #########
placed_quality_results = np.array([100, 100, 99, 94, 93, 92, 95, 100, 96]) #1-3 towel (trials 4-6), 4-6 waffle (trials 21-23), 7-9 checkered (trial 28, 30, 31)
placing_errors = 100-placed_quality_results
placing_str = ["d", "d", "d", "d", "r", "r", "r", "r", "r"]
placing_def_classes = ["A", "A", "A", "A", "B", "B", "B", "B", "B"]

piled_quality_results = np.array([95, 95, 99, 86, 96, 95, 22, 88, 95]) 
piling_errors = 100-piled_quality_results
piling_str = ["d", "d", "d", "r", "d", "r", "v", "r", "r"] 
piling_def_classes = ["A", "A", "A", "B", "A", "B", "C", "B", "B"]

labels = np.array(["dd", "dd", "dd", "rr"])

##################
errors = [placing_errors, piling_errors]
strategies = [placing_str, piling_str]
classes = [placing_def_classes, piling_def_classes]

delta=45

cost_tables = [place_initial_cost_table, pile_initial_cost_table]
costs_history = [[[[] for _ in range(3)] for _ in range(3)], [[[] for _ in range(3)] for _ in range(3)]] #cells evolution (matrix of vector of each cell)
cost_table_evolution = [[],[]] #matrix of matrices corresponding to the cost table at each timestep
changes=[[],[]] #values which change (to plot the circles)
changes[0].append(0) #cloth-to-table
changes[1].append(0) #cloth-to-cloth


def print_info(activate, arg1, arg2="", arg3="", arg4="", arg5="", arg6=""):
    if(activate):
        print(str(arg1) + str(arg2) + str(arg3) + str(arg4) + str(arg5) + str(arg6))

def plot_costs(axes, data, points):

    names = [['A vertical', 'A diagonal', 'A rotating'], ['B vertical', 'B diagonal', 'B rotating'], ['C vertical', 'C diagonal', 'C rotating']]
    colors = [['black', 'black', 'black'], ['cyan', 'green', 'pink'], ['blue', 'yellow', 'red']]
    plt.rc('axes', prop_cycle=cycler(color=plt.cm.hsv(np.linspace(0, 1, 9))))  # gama de colores automatica para las lineas

    x = np.arange(0, len(data[0][0])) #number of trials

    ## Plot place cost table updates
    for i in range(3): #rows
        for j in range(3): #columns 
            name = names[i][j]
            color = colors[i][j]
            axes.plot(x, data[i][j], linestyle='-', linewidth=2, label=name)  # Waypoints as red dots

    axes.scatter(x, points, color='black', zorder=3)

    plt.axvline(x=3, color='gray', linestyle='--', linewidth=2) #Towel to waffle
    plt.axvline(x=6, color='gray', linestyle='--', linewidth=2) #Waffle to checkered

    ## Plot config
    axes.set_xticks(x)
    axes.set_xlabel("Trial", fontsize=18)
    axes.grid()
    axes.grid(True,  linewidth=0.8, alpha=0.5)

def plot_quality(placed_quality_results, piled_quality_results, labels):
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
    fig = plt.figure(figsize=(9, 6))

    plt.scatter(x, piled_quality_results, color='black', zorder=3) 
    plt.plot(x_smooth, ypiled_smooth, color="royalblue", linewidth=2.5, linestyle="-", alpha=0.8) #"#FF5733 "#33CFFF"

    plt.axvline(x=3, color='gray', linestyle='--', linewidth=2) #Towel to waffle
    plt.axvline(x=6, color='gray', linestyle='--', linewidth=2) #Waffle to checkered

    plt.ylim(0,105) #Y axis range
    plt.xticks(x)
    plt.xlabel("Trial", fontsize=18)
    plt.ylabel("Pile quality (%)", fontsize=18)
    plt.title("Pile quality evolution over trials", fontsize=20, fontweight='bold', color="#333333")
    plt.grid(True,  linewidth=0.8, alpha=0.5)
    plt.gca().spines["top"].set_visible(False)
    plt.gca().spines["right"].set_visible(False)

    return fig

####################################################################################


for m in range(0,len(errors)): #cloth-table and cloth-cloth cost tables
    updater = cost_update.CostUpdater(cost_tables[m], 0.5, 0.3, 60, 0.5)
    costs_history[m] = updater.save_cell_evolution(cost_tables[m]) #Initialize cells
    cost_table_evolution[m].append(cost_tables[m].copy()) #Initialize cost tables history
    for n in range(0,len(placed_quality_results)): 
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
        costs_matrix = updater.get_cost_table()
        print_info(activate_print, "Previous cost for state-action ", def_class, "-", placing_str, " was ", costs_matrix[i,j])
        new_cost = updater.update_cost(i,j, errors[m][n], 8) # trial fixed to 8 to have a constant learning rate of 0.3 since these trials are executed after system adaptability experiments once the cost table is learned
        costs_matrix = updater.get_cost_table()
        print("TRIAL: ", n, " - Quality: ", 100-errors[m][n])
        print_info(activate_print, "New cost for state-action ", def_class, "-", placing_str, " is ", new_cost)
        print_info(True,"Updated Cost Table:\n", costs_matrix)

        costs_history[m] = updater.save_cell_evolution(costs_matrix)
        # print_info(activate_print,costs_history[m])
        changes[m].append(new_cost)
        cost_table_evolution[m].append(costs_matrix.copy())

        print_info(activate_print,"--------------------")
    print("---------------------------------------------")
    # print("Last cost table: \n", costs_matrix)
    print("---------------------------------------------")


plot_quality(placed_quality_results, piled_quality_results, labels)
# plt.show()

fig, axes = plt.subplots(nrows=2, ncols=1, figsize=(9, 8))  # Create a 2-row, 3-column figure

plot_costs(axes[0], costs_history[0], changes[0])
axes[0].set_ylabel("cloth-to-table cost", fontsize=18)

plot_costs(axes[1], costs_history[1], changes[1])
axes[1].set_ylabel("cloth-to-cloth cost", fontsize=18)

fig.suptitle("Cost update over trials", fontsize=20, fontweight='bold', color="#333333")
handles, labels = plt.gca().get_legend_handles_labels()  # Get all lines
fig.legend(handles[:9], labels[:9], title='State-Action Cost')  # # Def class and placing action combination cost - Show only the first 9
plt.show()