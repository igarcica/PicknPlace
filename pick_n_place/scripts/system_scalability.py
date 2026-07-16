
### SYSTEM SCALABILITY RESULTS
### This script plots the accumulated quality of the piled objects 

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.interpolate import CubicSpline
import cost_update as cost_update


## init cost table is previous learned one
place_initial_cost_table = np.array([ # Init cost table (placing error to minimize)
    [2, 2, 2],
    [6, 5, 5], 
    [1, 1, 1], 
])  
pile_initial_cost_table = np.array([ # Init cost table (placing error to minimize)
    [9, 4, 2],
    [27, 5, 2], 
    [26, 10, 11], 
])  

################## SYSTEM'S SCALABILITY ##################
######### TOWEL 8l - pile of 4 objects ######### - 
all_quality_results = [
    np.array([99, 96, 95, 98]), #1.1 to 1.4
    np.array([99, 97, 98, 98]), #3.1 to 3.4
    np.array([99, 98, 92, 94]),  #7.1 to 7.4
    np.array([98, 93, 93, 96]),  #10.1 to 10.4
    # np.array([98, 95, 86, 89])  #13.1 to 13.4
    np.array([93, 96, 90, 89])  #14.1 to 14.4
]
all_placing_quality = np.array([arr[0] for arr in all_quality_results]) #1.1, 3.1, 7.1, 10.1
all_piling_quality = np.concatenate([arr[1:] for arr in all_quality_results]) #1.2-1.4, 3.2-3.4, 7.2-7.4, 10.2-10-4
all_placing_errors = 100 - all_placing_quality
all_piling_errors = 100 - all_piling_quality
errors = [all_placing_errors, all_piling_errors]

print(all_placing_errors)
print(all_piling_errors)

all_str = [
    ["v", "r", "r", "d"],
    ["v", "d", "d", "d"],
    ["v", "d", "d", "r"],
    ["v", "d", "r", "d"],
    ["v", "d", "d", "r"]
]
all_placing_str = np.array([arr[0] for arr in all_str]) #1.1, 3.1
all_piling_str = np.concatenate([arr[1:] for arr in all_str]) #1.2-1.4, 3.2-3.4
strategies = [all_placing_str, all_piling_str]

all_def_classes = [
    ["A", "A", "A", "A"],
    ["A", "A", "A", "A"],
    ["A", "A", "A", "A"],
    ["A", "A", "A", "A"],
    ["C", "A", "A", "A"]
]
all_placing_def_classes = np.array([arr[0] for arr in all_def_classes]) #1.1, 3.1
all_piling_def_classes = np.concatenate([arr[1:] for arr in all_def_classes]) #1.2-1.4, 3.2-3.4
classes = [all_placing_def_classes, all_piling_def_classes]

# accumulated_quality = np.cumsum(quality_results)
# indices = np.arange(1, len(accumulated_quality) + 1) # Accumulated error of pile
# norm_accum_quality = accumulated_quality / indices # percentage (%)

# placing_errors = 100-norm_accum_quality
# placing_errors = 100-quality_results
# placing_str = ["v", "d", "d", "r"]
# placing_def_classes = ["A", "A", "A", "A"]

# ##################
# # placed_object = 0
# piled_objects = len(quality_results)
# # print(placed_object)
# # print(piled_objects)

##borrar
# errors = [[placing_errors[0]], placing_errors[1:piled_objects]]
# print("Errors: ", errors)
# strategies = [placing_str[0], placing_str[1:piled_objects]]
# classes = [placing_def_classes[0], placing_def_classes[1:piled_objects]]
#-

# delta=45

cost_tables = [place_initial_cost_table, pile_initial_cost_table]
costs_history = [[[[] for _ in range(3)] for _ in range(3)], [[[] for _ in range(3)] for _ in range(3)]] #cells evolution (matrix of vector of each cell)
cost_table_evolution = [[],[]] #matrix of matrices corresponding to the cost table at each timestep
changes=[[],[]] #values which change (to plot the circles)
changes[0].append(0) #cloth-to-table
changes[1].append(0) #cloth-to-cloth


def plot_quality(data, ax):

        # Create a smooth parameterized curve using cubic splines
        x = np.arange(1, len(data)+1) #np.linspace(0,1, len(x))
        t = np.linspace(0, 1, len(x))  # Normalized parameter
        cs_x = CubicSpline(t, x)  # X interpolation
        cs_ypiled = CubicSpline(t, data)  # Y interpolation

        # Generate fine-grained trajectory points
        t_fine = np.linspace(0, 1, 100)
        x_smooth = cs_x(t_fine)
        ypiled_smooth = cs_ypiled(t_fine)

        # Plot the trajectory
        # fig = plt.figure(figsize=(9, 6))

        ax.scatter(x, data, color='black', zorder=3) 
        ax.plot(x_smooth, ypiled_smooth, color="royalblue", linewidth=2.5, linestyle="-", alpha=0.8) #"#FF5733 "#33CFFF"

        # #Put labels to points
        # for i,j in zip(x,placed_data):
        #     plt.annotate(labels[i], (i+0.05,j+0.05))

        ax.set_title(f"Trial {i+1}")
        ax.set_ylabel("Quality (%)")
        ax.set_ylim(0, 105)
        ax.set_xticks(x)

        return fig


####################################################################################
##------Plot quality of each object in the pile------

# plot_quality(norm_accum_quality)
# plt.show()

# quality_results = [quality_results_1, quality_results_2]
# print("size", len(quality_results))
# fig, axs = plt.subplots(len(quality_results))
n = len(all_quality_results) #number of trials
fig, axes = plt.subplots(n, 1, figsize=(9, 3*n), sharex=True)
for i, (ax, quality_results) in enumerate(zip(axes, all_quality_results)):
    plot_quality(quality_results, ax)

axes[-1].set_xlabel("Index")

# plt.ylim(0,105)
# plt.xticks(x)
# plt.xlabel("Object", fontsize=18)
# plt.ylabel("Pile quality (%)", fontsize=18)
# plt.title("Pile quality evolution over trials", fontsize=20, fontweight='bold', color="#333333")
# plt.grid(True,  linewidth=0.8, alpha=0.5)
# plt.gca().spines["top"].set_visible(False)
# plt.gca().spines["right"].set_visible(False)

plt.tight_layout()
plt.show()



####################################################################################
## Update cost after placing each PLACED object in the pile
for m in range(0,len(errors)):  #Update the two tables
    print("m", m)
    updater = cost_update.CostUpdater(cost_tables[m], 0.5, 0.3, 60, 0.5)
    costs_history[m] = updater.save_cell_evolution(cost_tables[m]) #Initialize cells
    cost_table_evolution[m].append(cost_tables[m].copy()) #Initialize cost tables history
    for n in range(0,len(errors[m])): # Consider all objects (first placed object and following piled objects)
        print("n", n)
        def_class = classes[m][n]
        placing_str = strategies[m][n]
        print("def_class ", def_class)
        print("placing str: ", placing_str)
        
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
        hola = updater.update_cost(i,j, errors[m][n], 8)
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