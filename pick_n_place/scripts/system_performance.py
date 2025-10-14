
### SYSTEM PERFORMANCE RESULTS
### This script plots the place and pile quality of the system's performance experiments (piles of 2 objects)
### Objects: waffle rag 8l, chekered rag 6l (folded in thirds + halves)

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.interpolate import CubicSpline
from scipy.interpolate import PchipInterpolator
import cost_update as cost_update
from cycler import cycler

activate_print=True

# ## init cost table is the previous learned one from system's adaptability experiments
# place_initial_cost_table = np.array([ # Init cost table (placing error to minimize)
#     [17, 1, 1],
#     [8, 22, 6], 
#     [30, 25, 6], 
# ])  
# pile_initial_cost_table = np.array([ # Init cost table (placing error to minimize)
#     [7, 3, 4],
#     [30, 14, 9], 
#     [30, 30, 30], 
# ])  

## init cost table is the previous learned one from system's adaptability2 experiments
place_initial_cost_table = np.array([ # Init cost table (placing error to minimize)
    [2, 2, 2],
    [5, 4, 4], 
    [1, 1, 1]
])  
pile_initial_cost_table = np.array([ # Init cost table (placing error to minimize)
    [9, 4, 3],
    [27, 4, 4], 
    [26, 10, 11]
])  

# ################## SYSTEM'S PERFORMANCE ##################
# # ######### WAFFLE RAG 8L #########
# placed_quality_results = np.array([0, 99, 94, 93, 92]) #trials 20 to 23
# placing_errors = 100-placed_quality_results
# placing_str = ["0", "d", "d", "r", "r"]
# placing_def_classes = ["0", "A", "A", "B", "B"]

# piled_quality_results = np.array([0, 81, 86, 96, 95]) #trials 20 to 23
# piling_errors = 100-piled_quality_results
# piling_str = ["0", "r", "r", "d", "r"] 
# piling_def_classes = ["0", "B", "B", "A", "B"]

# labels = np.array(["0", "dr"])

# # # ######### CHECKERED RAG 6L #########
# # placed_quality_results = np.array([0, 96]) #trials 26
# # placing_errors = 100-placed_quality_results
# # placing_str = ["0", "r"]
# # placing_def_classes = ["0", "B"]

# # piled_quality_results = np.array([0, 95]) #trials 26
# # piling_errors = 100-piled_quality_results
# # piling_str = ["0", "r"] 
# # piling_def_classes = ["0", "B"]

# # labels = np.array(["0", "rr"])


# ######### ALL - TOWEL 8L, WAFFLE 8L, CHEKERED 6L #########
# placed_quality_results = np.array([100, 100, 99, 94, 93, 92, 95, 100, 96]) #1-3 towel (trials 4-6), 4-6 waffle (trials 21-23), 7-9 checkered (trial 28, 30, 31)
# placing_errors = 100-placed_quality_results
# placing_str = ["d", "d", "d", "d", "r", "r", "r", "r", "r"]
# placing_def_classes = ["A", "A", "A", "A", "B", "B", "B", "B", "B"]

# piled_quality_results = np.array([95, 95, 99, 86, 96, 95, 22, 88, 95]) 
# piling_errors = 100-piled_quality_results
# piling_str = ["d", "d", "d", "r", "d", "r", "v", "r", "r"] 
# piling_def_classes = ["A", "A", "A", "B", "A", "B", "C", "B", "B"]

# labels = np.array(["dd", "dd", "dd", "rr"])


################## SYSTEM'S PERFORMANCE2 ##################
# ######### Towel and towel (system performance trials 4 to 6) + Checkered 8l and Waffle 8L (trials 1 to 3) + ######### NO
# ######### Towel and towel (trials  to ) + cotton napkin and cotton napkin + Checkered 8l and Waffle 8L () + towel and napkin ######### 
# placed_quality_results = np.array([0, 99, 98, 99, 95, 94, 90]) #Previous placing quality metric
## Planner
# placed_quality_results = np.array([0, 97, 97, 99, 96, 94, 93, 98, 98, 98]) #planner - towel, cotnap, towel+pillowc
# placing_str = ["0", "v", "v", "v", "d", "d", "r", "v", "v", "v"]
# placing_def_classes = ["0", "A", "A", "A", "B", "B", "B", "A", "A", "A"]
##Reactive
placed_quality_results = np.array([0, 97, 98, 98, 93, 59, 63]) 
placing_str = ["0", "v", "v", "v", "v", "v", "d"]
placing_def_classes = ["0", "A", "A", "A", "C", "C", "C"]

placing_errors = 100-placed_quality_results

# piled_quality_results = np.array([0, 96, 97, 97, 92, 98, 97]) #Previous placing quality metric
##Planner
# piled_quality_results = np.array([0, 99, 98, 98, 92, 98, 97, 95, 99, 98]) #planner
# piling_str = ["0", "r", "r", "r", "d", "r", "r", "r", "r", "r", "r"] #Planner
# piling_def_classes = ["0", "A", "A", "A", "B", "B", "B", "B", "B", "B", "B"] #Planner
## Reactive
piled_quality_results = np.array([0, 97, 96, 98, 68, 17, 18]) #Reactive
piling_str = ["0", "r", "r", "r", "d", "r", "d"] #Reactive
piling_def_classes = ["0", "A", "A", "A", "C", "C", "C"] #Reactive

piling_errors = 100-piled_quality_results

labels = np.array(["0", "vr", "vr", "vr", "dd", "dr", "rr"])


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

# def plot_quality(placed_quality_results, piled_quality_results, labels):
#     # Create a smooth parameterized curve using cubic splines
#     x = np.arange(0, len(placed_quality_results)) #np.linspace(0,1, len(x))
#     t = np.linspace(0, 1, len(x))  # Normalized parameter
#     cs_x = CubicSpline(t, x)  # X interpolation
#     cs_yplaced = CubicSpline(t, placed_quality_results)  # Y interpolation
#     cs_ypiled = CubicSpline(t, piled_quality_results)  # Y interpolation

#     # Generate fine-grained trajectory points
#     t_fine = np.linspace(0, 1, 100)
#     x_smooth = cs_x(t_fine)
#     yplaced_smooth = cs_yplaced(t_fine)
#     ypiled_smooth = cs_ypiled(t_fine)

#     # Plot the trajectory
#     fig = plt.figure(figsize=(9, 6))

#     plt.scatter(x, piled_quality_results, color='black', zorder=3) 
#     plt.plot(x_smooth, ypiled_smooth, color="royalblue", linewidth=2.5, linestyle="-", alpha=0.8) #"#FF5733 "#33CFFF"

#     plt.axvline(x=3, color='gray', linestyle='--', linewidth=2) #Towel to waffle
#     plt.axvline(x=6, color='gray', linestyle='--', linewidth=2) #Waffle to checkered

#     plt.ylim(0,105) #Y axis range
#     plt.xticks(x)
#     plt.xlabel("Trial", fontsize=18)
#     plt.ylabel("Pile quality (%)", fontsize=18)
#     plt.title("Pile quality evolution over trials", fontsize=20, fontweight='bold', color="#333333")
#     plt.grid(True,  linewidth=0.8, alpha=0.5)
#     plt.gca().spines["top"].set_visible(False)
#     plt.gca().spines["right"].set_visible(False)

#     return fig

def plot_quality(placed_quality_results, piled_quality_results, labels):

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
        plt.plot(x_smooth, yplaced_smooth, 'g-', label="TAMP", linewidth=3, alpha=0.8)
        plt.scatter(x, piled_quality_results, color='black', zorder=3)
        plt.plot(x_smooth, ypiled_smooth, color="royalblue", linestyle="-", label="Reactive planner", linewidth=3, alpha=0.8)

        plt.axvline(x=3.5, color='gray', linestyle='--', linewidth=2) #Towels
        plt.axvline(x=6.5, color='gray', linestyle='--', linewidth=2) #Cotton napkins

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
        plt.text(0.35, -0.07, "Piles of towels", ha="center", transform=plt.gca().transAxes, fontsize=12)
        plt.text(0.83, -0.07, "Piles of cotton napk", ha="center", transform=plt.gca().transAxes, fontsize=12)

        # plt.show()

        return fig
####################################################################################


for m in range(0,len(errors)): #cloth-table and cloth-cloth cost tables
    updater = cost_update.CostUpdater(cost_tables[m], 0.5, 0.3, 60, 0.5)
    costs_history[m] = updater.save_cell_evolution(cost_tables[m]) #Initialize cells
    cost_table_evolution[m].append(cost_tables[m].copy()) #Initialize cost tables history
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


# plot_quality(placed_quality_results, piled_quality_results, labels)
# plt.show()

# fig, axes = plt.subplots(nrows=2, ncols=1, figsize=(9, 8))  # Create a 2-row, 3-column figure

# plot_costs(axes[0], costs_history[0], changes[0])
# axes[0].set_ylabel("cloth-to-table cost", fontsize=18)

# plot_costs(axes[1], costs_history[1], changes[1])
# axes[1].set_ylabel("cloth-to-cloth cost", fontsize=18)

# fig.suptitle("Cost update over trials", fontsize=20, fontweight='bold', color="#333333")
# handles, labels = plt.gca().get_legend_handles_labels()  # Get all lines
# fig.legend(handles[:9], labels[:9], title='State-Action Cost')  # # Def class and placing action combination cost - Show only the first 9
# plt.show()

#############################
##Planner
placed_quality_results_planner = np.array([0, 97, 97, 99, 96, 94, 93, 98, 98, 98]) #planner
piled_quality_results_planner = np.array([0, 99, 98, 98, 92, 98, 97, 95, 99, 98]) #planner
## REactive
placed_quality_results_reactive = np.array([0, 97, 98, 98, 98, 59, 63, 0, 0, 0]) 
piled_quality_results_reactive = np.array([0, 97, 96, 98, 65, 17, 18, 0, 0, 0]) #Reactive

plot_quality(piled_quality_results_planner, piled_quality_results_reactive, labels)
plt.show()


############################### Plot cost evolution as table

# labels
def_classes = ["A", "B", "C"]
placing_str = ["vertical", "diagonal", "rotating"]

#### cloth-to-table costs
arr = costs_history[0]
rows = [] # build rows
for i, dclass in enumerate(def_classes):
    for j, place in enumerate(placing_str):
        rows.append([dclass, place] + arr[i][j])

# create dataframe
df1 = pd.DataFrame(rows, columns=["Def class", "Placing str"] + [f"trial {k}" for k in range(len(arr[0][0]))])

# plot table
fig, ax = plt.subplots(figsize=(12, 4))
ax.axis("off")
table = ax.table(
    cellText=df1.values,
    colLabels=df1.columns,
    loc="center",
    cellLoc="center"
)
table.auto_set_font_size(False)
table.set_fontsize(10)
table.scale(1.2, 1.2)

plt.show()

#### cloth-to-cloth costs
arr = costs_history[1]
rows = []
for i, dclass in enumerate(def_classes):
    for j, place in enumerate(placing_str):
        rows.append([dclass, place] + arr[i][j])

# create dataframe
df2 = pd.DataFrame(rows, columns=["Def class", "Placing str"] + [f"trial {k}" for k in range(len(arr[0][0]))])

# plot table
fig, ax = plt.subplots(figsize=(12, 4))
ax.axis("off")
table = ax.table(
    cellText=df2.values,
    colLabels=df2.columns,
    loc="center",
    cellLoc="center"
)
table.auto_set_font_size(False)
table.set_fontsize(10)
table.scale(1.2, 1.2)

plt.show()

print(df1)
print("-------------------------------------------------")
print(df2)