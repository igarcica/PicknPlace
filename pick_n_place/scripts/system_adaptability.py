### SYSTEM ADAPTABILITY RESULTS
### This script collects the placing_costs_update.py and the plot_placing_quality.py

import numpy as np
import pandas as pd
import statistics as sts
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.interpolate import make_interp_spline
import cost_update as cost_update

## UTIL FUNCTIONS
activate_print=False

def print_info(activate, arg1, arg2="", arg3="", arg4="", arg5="", arg6=""):
    if(activate):
        print(str(arg1) + str(arg2) + str(arg3) + str(arg4) + str(arg5) + str(arg6))

####################################################################################
### INPUT DATA

## ---- Placing cost table ----
# Def clas | Vertical | Diagonal | Rotating
#    A     |    0     |    0     |    0
#    B     |    0     |    0     |    0
#    C     |    0     |    0     |    0
# place_initial_cost_table = np.array([ # Init cost table (placing error to minimize)
#     [17, 0, 0],
#     [8, 22, 6], 
#     [30, 25, 6], 
# ])  
# pile_initial_cost_table = np.array([ # Init cost table (placing error to minimize)
#     [0, 0, 0],
#     [30, 14, 9], 
#     [30, 30, 30], 
# ]) 
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

####################################################################################

#### PILLOWCASE ####
# placed_quality_results = np.array([0, 13, 49, 88, 94, 65, 56, 94, 97, 91, 90, 95, 96, 83, 86, 89, 96,94,  88,  94, 92]) #exp31(t=10) bit changed
# placing_errors = 100-placed_quality_results
# placing_str = ["0", "v", "d", "r", "v", "v", "d", "r", "r", "r", "v", "r", "r", "r", "v", "v", "r", "r", "r", "r", "r"]
# placing_def_classes = ["0", "C", "C", "C", "B", "A", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B"]

# piled_quality_results = np.array([0, 0, 0, 8, 31, 81, 86, 78, 78, 93, 78, 79, 85, 93, 77, 88, 93,91, 85,  92, 91]) #exp31(t=10) bit changed
# piling_errors = 100-piled_quality_results
# piling_str = ["0", "v", "d", "r", "v", "d", "r", "r", "d", "r", "r", "d", "r", "r", "r", "r", "r", "r", "r", "r", "r"] 
# piling_def_classes = ["0", "C", "C", "C", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B"]

# labels = np.array(["0", "vv", "dd", "rr", "vv", "vd", "dr", "rr", "rd", "rr", "vr", "rd", "rr", "rr", "vr", "vr", "rr", "rr", "rr", "rr", "rr"])

# print(sts.mean([0,0,8]))
# print(sts.mean([77, 88, 91]))

######### TOWEL #########
# placed_quality_results = np.array([0, 92, 92, 94, 96]) 
# placing_errors = 100-placed_quality_results
# placing_str = ["0", "v", "d", "r", "r"]
# placing_def_classes = ["0", "A", "A", "A", "A"]
# piled_quality_results = np.array([0, 70, 89, 80, 91]) 
# piling_errors = 100-piled_quality_results
# piling_str = ["0", "v", "d", "r", "d"] 
# piling_def_classes = ["0", "A", "A", "A", "A"]

# labels = np.array(["0", "vv", "dd", "rr", "rd"])

##Starting with learned cost table from pillowcase
# placed_quality_results = np.array([0, 98, 99, 96, 99, 99, 98, 99, 98]) #95, 97, 96, 96, 96]) 
# placing_errors = 100-placed_quality_results
# placing_str = ["0", "d", "r", "r", "d", "d", "d", "d", "d"]
# placing_def_classes = ["0", "A", "A", "A", "A", "A", "A", "A", "A"]
# piled_quality_results = np.array([0, 76, 88, 91, 90, 94, 95, 86, 90])#29, 68, 73, 70, 91]) 
# piling_errors = 100-piled_quality_results
# piling_def_classes = ["0", "A", "A", "A", "A", "A", "A", "A", "A"]
# piling_str = ["0", "v", "d", "r", "r", "d", "d", "d", "d"] 

# labels = np.array(["0", "dv", "rd", "rr", "dr", "rd", "dd", "dd", "dd", "dd"])

######### PILLOWCASE ONLY LONG EDGE #########
# placed_quality_results = np.array([0, 13, 49, 88, 91, 83, 91,88, 78, 92]) 
# placing_errors = 100-placed_quality_results
# placing_str = ["0", "v", "d", "r", "r", "r", "r", "r", "r", "r"]
# placing_def_classes = ["0", "C", "C", "C", "C", "C", "C", "C", "C", "C"]
# piled_quality_results = np.array([0, 0,0,8,42,43,59, 0, 22, 33]) 
# piling_errors = 100-piled_quality_results
# piling_str = ["0", "v", "d", "r", "v", "d", "r", "r", "d", "v"] 
# piling_def_classes = ["0", "C", "C", "C", "C", "C", "C", "C", "C", "C"]
# labels = np.array(["0", "vv", "dd", "rr", "rv", "rd", "rr", "rr", "rd", "rv"])


################## SYSTEM'S ADAPTABILITY ##################
######### PILLOWCASE + TOWEL #########
# placed_quality_results = np.array([0, 13, 49, 88, 94, 65, 56, 94, 97, 91, 90, 95, 96, 83, 86, 89, 96, 94, 88, 94, 92, 98, 99, 96, 99, 99, 98]) #exp31(t=10) bit changed
# placing_errors = 100-placed_quality_results
# placing_str = ["0", "v", "d", "r", "v", "v", "d", "r", "r", "r", "v", "r", "r", "r", "v", "v", "r", "r", "r", "r", "r", "d", "r", "r", "d", "d", "d", "d"]
# placing_def_classes = ["0", "C", "C", "C", "B", "A", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "A", "A", "A", "A", "A", "A", "A"]

# piled_quality_results = np.array([0, 0, 0, 8, 31, 81, 86, 78, 78, 93, 78, 79, 85, 93, 77, 88, 93, 91, 85, 92, 91, 76, 88, 91, 90, 94, 95]) #exp31(t=10) bit changed
# piling_errors = 100-piled_quality_results
# piling_str = ["0", "v", "d", "r", "v", "d", "r", "r", "d", "r", "r", "d", "r", "r", "r", "r", "r", "r", "r", "r", "r", "v", "d", "r", "r", "d", "d", "d"] 
# piling_def_classes = ["0", "C", "C", "C", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "A", "A", "A", "A", "A", "A", "A"]

# labels = np.array(["0", "vv", "dd", "rr", "vv", "vd", "dr", "rr", "rd", "rr", "vr", "rd", "rr", "rr", "vr", "vr", "rr", "rr", "rr", "rr", "rr", "dv", "rd", "rr", "dr", "rd", "dd", "dd", "dd"])

################## SYSTEM'S ADAPTABILITY ################## notion exps values
# ######### PILLOWCASE + TOWEL #########
# placed_quality_results = np.array([0, 13, 49, 88, 94, 65, 56, 94, 97, 91, 90, 95, 96, 83, 86, 89, 96, 94, 88, 94, 92, 96, 99, 96, 99, 99, 98]) #trials 21 y 23 lowered wrt notion values (from 98 to 96)
# placing_errors = 100-placed_quality_results
# placing_str = ["0", "v", "d", "r", "v", "v", "d", "r", "r", "r", "v", "r", "r", "r", "v", "v", "r", "r", "r", "r", "r", "d", "r", "r", "d", "d", "d"]
# placing_def_classes = ["0", "C", "C", "C", "B", "A", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "A", "A", "A", "A", "A", "A"]

# piled_quality_results = np.array([0, 0, 0, 8, 31, 81, 86, 78, 78, 93, 78, 79, 85, 93, 77, 88, 93, 91, 85, 92, 91, 76, 88, 91, 90, 94, 95]) 
# piling_errors = 100-piled_quality_results
# piling_str = ["0", "v", "d", "r", "v", "d", "r", "r", "d", "r", "r", "d", "r", "r", "r", "r", "r", "r", "r", "r", "r", "v", "d", "r", "r", "d", "d"] 
# piling_def_classes = ["0", "C", "C", "C", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "B", "A", "A", "A", "A", "A", "A"]

# labels = np.array(["0", "vv", "dd", "rr", "vv", "vd", "dr", "rr", "rd", "rr", "vr", "rd", "rr", "rr", "vr", "vr", "rr", "rr", "rr", "rr", "rr", "dv", "rd", "rr", "dr", "dd", "dd"])



################## SYSTEM'S ADAPTABILITY 2 ##################
placed_quality_results = np.array([0, 98, 96, 97, 83, 96, 97, 95, 94, 92, 93, 95, 97, 96, 97, 97]) #94, 97, 96, 97, 100])#, 95]) 
#exps: 4.1 to 6.2 of system performance
######### CHECKERED 8L ######### - 
placing_errors = 100-placed_quality_results
placing_str = ["0", "v", "d", "r", "v", "d", "r", "d", "r", "d", "r", "v", "d", "r", "d", "r"]#, "d"]
placing_def_classes = ["0", "C", "C", "C", "B", "B", "B", "B", "B", "B", "B", "A", "A", "A", "A", "A"]

piled_quality_results = np.array([0, 12, 67, 64, 0, 87, 92, 98, 97, 94, 95, 69, 95, 92, 93, 95])#, 93]) 
piling_errors = 100-piled_quality_results
piling_str = ["0", "v", "d", "r", "v", "d", "r", "r", "r", "r", "r", "v", "d", "r", "d", "r"]#, "2d"]
piling_def_classes = ["0", "C", "C", "C", "B", "B", "B", "B", "B", "B", "B", "A", "A", "A", "A", "A"]#, "A"]
labels = np.array(["0", "vv", "dd", "rr", "vv", "dd", "rr", "dr", "rr", "rr", "dr", "vv", "dd", "rr", "dd", "rr"])#, "dd"])



##########################################
errors = [placing_errors, piling_errors]
strategies = [placing_str, piling_str]
classes = [placing_def_classes, piling_def_classes]



delta=45
# print("Std deviation: ", sts.stdev(placing_errors))
# print("Std deviation: ", sts.stdev(piling_errors))



###################################3
cost_tables = [place_initial_cost_table, pile_initial_cost_table]
costs_history = [[[[] for _ in range(3)] for _ in range(3)], [[[] for _ in range(3)] for _ in range(3)]] #cells evolution (matrix of vector of each cell)
cost_table_evolution = [[],[]] #matrix of matrices corresponding to the cost table at each timestep
#
changes=[[],[]] #values which change (to plot the circles)
# #Start with table costs
# for m in range(2): #placing and piling costs
#     for i in range(3):
#         for j in range(3):
#             changes[m][i][j].append(matrix[i, j]) ## Save costs in separated arrays to be plotted
changes[0].append(0) #cloth-to-table
changes[1].append(0) #cloth-to-cloth
# costs_history[0] = updater.save_cell_evolution(place_initial_cost_table)
# costs_history[1] = updater.save_cell_evolution(pile_initial_cost_table)
# updater = CostUpdater(place_initial_cost_table, 0.5, 0.3, 45, 0.5)

# cost_table_evolution #matrix of matrices corresponding to the cost table at each timestep

# n_exp = 1

for m in range(0,len(errors)):
    updater = cost_update.CostUpdater(cost_tables[m], 0.5, 0.3, 90, 0.5)
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
        hola = updater.update_cost(i,j, errors[m][n], n)
        costs_matrix = updater.get_cost_table()
        print_info(True,"Updated Cost Table:\n", costs_matrix)

        costs_history[m] = updater.save_cell_evolution(costs_matrix)
        print_info(activate_print,costs_history[m])
        changes[m].append(hola)
        cost_table_evolution[m].append(costs_matrix.copy())

        print("----------")
    print("---------------------------------------------")
    print("Last cost table: \n", costs_matrix)
    print("---------------------------------------------")

############################

# print("Final PILING cost table: ")
# print(costs_history[1])
updater.plot_quality(placed_quality_results, piled_quality_results, labels)
plt.axvline(x=10, color='gray', linestyle='--', linewidth=2) #Plot a dashed line when switching between checkered and 

# fig, axes = plt.subplots(nrows=2, ncols=1, figsize=(9, 8))  # Create a 2-row, 3-column figure
# # fig = plt.figure(figsize=(8, 6))
# updater.plot_costs(axes[0], costs_history[0], changes[0])
# axes[0].set_ylabel("cloth-to-table cost", fontsize=18)
# axes[0].axvline(x=10, color='gray', linestyle='--', linewidth=2) #Plot a dashed line when switching between checkered and 
# # plt.title("PLACE Cost update evolution")

# updater.plot_costs(axes[1], costs_history[1], changes[1])
# axes[1].set_ylabel("cloth-to-cloth cost", fontsize=18)
# axes[1].axvline(x=10, color='gray', linestyle='--', linewidth=2) #Plot a dashed line when switching between checkered and 
# # plt.title("PILE Cost update evolution")

# fig.suptitle("Cost update over trials", fontsize=20, fontweight='bold', color="#333333")
# handles, labels = plt.gca().get_legend_handles_labels()  # Get all lines
# fig.legend(handles[:9], labels[:9], title='State-Action Cost')  # # Def class and placing action combination cost - Show only the first 9
plt.show()


# ##########################################################################################
# ###### ANIMATED PLOTS
# ### ----Placing quality evolution----

# # x = np.linspace(0, 27, len(piled_quality_results))

# # # Interpolation
# # x_smooth = np.linspace(x.min(), x.max(), 500)
# # spline = make_interp_spline(x, piled_quality_results, k=3)
# # y_smooth = spline(x_smooth)

# # # Setup
# # fig, ax = plt.subplots(figsize=(10, 6))
# # line, = ax.plot([], [], lw=2, color='#1f77b4')
# # point, = ax.plot([], [], 'ro')
# # scatter = ax.scatter([], [], color='black', s=50, zorder=3)  # Hollow circles

# # ax.set_xlim(x.min(), x.max())
# # ax.set_ylim(piled_quality_results.min() - 10, piled_quality_results.max() + 10)

# # # plt.axvline(x=x[20], color='gray', linestyle='--', linewidth=2) #Plot a dashed line when plotting the pillowcase + towel results

# # # Styling
# # ax.set_xlabel("Trial", fontsize=18)
# # ax.set_ylabel("Pile quality (%)", fontsize=18)
# # ax.set_title("Pile quality evolution over trials", fontsize=20, fontweight='bold', color="#333333")
# # ax.grid(True, linewidth=0.8, alpha=0.5)
# # ax.spines["top"].set_visible(False)
# # ax.spines["right"].set_visible(False)

# # # Example custom labels (must match length of x)
# # custom_labels = [f"{i}" for i in range(len(x))]  # e.g., T0, T1, ..., T26
# # ax.set_xticks(x) # Set ticks at original data point positions
# # ax.set_xticklabels(custom_labels, fontsize=10)  # Rotate for readability


# # # Animation
# # def init_plot_quality_anim():
# #     line.set_data([], [])
# #     point.set_data([], [])
# #     scatter.set_offsets([])
# #     return line, point, scatter

# # def update_plot_quality_anim(frame):
# #     # Curve line + red dot
# #     line.set_data(x_smooth[:frame], y_smooth[:frame])
# #     point.set_data(x_smooth[frame - 1], y_smooth[frame - 1])

# #     # Show markers up to the current x_smooth point
# #     current_x = x_smooth[frame - 1]
# #     visible_indices = np.where(x <= current_x)[0]
# #     scatter.set_offsets(np.column_stack((x[visible_indices], piled_quality_results[visible_indices])))

# #     return line, point, scatter

# # # Animate
# # ani = FuncAnimation(fig, update_plot_quality_anim, frames=len(x_smooth), init_func=init_plot_quality_anim,
# #                     blit=False, interval=15, repeat=False)

# # plt.tight_layout()

# # # Save as video
# # ani.save("pile_quality_animation.mp4", writer="ffmpeg", fps=30)
# # plt.show()



# ##########################################################################################
# ######## ---Cost table evolution 

# # matrices=cost_table_evolution[1]
# # nrows, ncols = matrices[0].shape
# # print(len(matrices))
# # print(matrices[0])

# # # Create plot
# # fig, ax = plt.subplots()
# # heatmap = ax.imshow(matrices[0], cmap='YlOrRd', vmin=0, vmax=50)

# # # Color bar
# # cbar = plt.colorbar(heatmap, ax=ax)
# # cbar.set_label("Value", fontsize=12)

# # # Axis styling
# # ax.set_xticks(np.arange(ncols))
# # ax.set_yticks(np.arange(nrows))
# # ax.set_xticklabels(['Vertical', 'Diagonal', 'Rotating'])
# # ax.set_yticklabels(['A', 'B', 'C'])
# # ax.set_title("Cost Evolution", fontsize=16, fontweight='bold')

# # # Grid lines
# # for edge, spine in ax.spines.items():
# #     spine.set_visible(False)
# # ax.set_xticks(np.arange(-.5, ncols, 1), minor=True)
# # ax.set_yticks(np.arange(-.5, nrows, 1), minor=True)
# # ax.grid(which='minor', color='gray', linestyle='-', linewidth=1)
# # ax.tick_params(which='minor', bottom=False, left=False)

# # # Initialize cell labels
# # cell_texts = [[ax.text(j, i, "", ha="center", va="center", color="black", fontsize=12)
# #                for j in range(ncols)] for i in range(nrows)]

# # # Animation update
# # def update(frame):
# #     data = matrices[frame]
# #     heatmap.set_data(data)
# #     for i in range(nrows):
# #         for j in range(ncols):
# #             cell_texts[i][j].set_text(f"{data[i, j]}")
# #     return [heatmap] + [text for row in cell_texts for text in row]

# # # Animate
# # ani = FuncAnimation(fig, update, frames=len(matrices), interval=1000, repeat=False)

# # # ani.save("cost_update.mp4", writer="ffmpeg", fps=30)
# # plt.show()










# ########### MATRIX EVOLUTION WITH MOUSE CLICKS
# matrices=cost_table_evolution[0]
# nrows, ncols = matrices[0].shape

# # Create plot
# fig, ax = plt.subplots()
# heatmap = ax.imshow(matrices[0], cmap='YlOrRd', vmin=0, vmax=50)

# # Add color bar
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

# # --- Add text annotations ---
# text_objects = []
# for i in range(nrows):
#     row_texts = []
#     for j in range(ncols):
#         val = matrices[0][i, j]
#         text = ax.text(j, i, f'{val:.1f}', ha='center', va='center', color='black')
#         row_texts.append(text)
#     text_objects.append(row_texts)

# # --- Interaction logic ---
# current_index = [0]  # Use a mutable type to allow modification inside event handler

# def on_click(event):
#     if event.button == 1:  # Left mouse button
#         current_index[0] += 1
#         if current_index[0] >= len(matrices):
#             current_index[0] = 0  # Loop back to beginning
#         new_matrix = matrices[current_index[0]]
#         heatmap.set_data(new_matrix)
        
#         # Update text values
#         for i in range(nrows):
#             for j in range(ncols):
#                 text_objects[i][j].set_text(f'{new_matrix[i, j]:.1f}')
        
#         fig.canvas.draw_idle()

# # Connect the click event
# fig.canvas.mpl_connect('button_press_event', on_click)

# plt.show()


# #################################
# matrices=cost_table_evolution[1]
# nrows, ncols = matrices[0].shape

# # Create plot
# fig, ax = plt.subplots()
# heatmap = ax.imshow(matrices[0], cmap='YlOrRd', vmin=0, vmax=50)

# # Add color bar
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

# # --- Add text annotations ---
# text_objects = []
# for i in range(nrows):
#     row_texts = []
#     for j in range(ncols):
#         val = matrices[0][i, j]
#         text = ax.text(j, i, f'{val:.1f}', ha='center', va='center', color='black')
#         row_texts.append(text)
#     text_objects.append(row_texts)

# # --- Interaction logic ---
# current_index = [0]  # Use a mutable type to allow modification inside event handler

# def on_click(event):
#     if event.button == 1:  # Left mouse button
#         current_index[0] += 1
#         if current_index[0] >= len(matrices):
#             current_index[0] = 0  # Loop back to beginning
#         new_matrix = matrices[current_index[0]]
#         heatmap.set_data(new_matrix)
        
#         # Update text values
#         for i in range(nrows):
#             for j in range(ncols):
#                 text_objects[i][j].set_text(f'{new_matrix[i, j]:.1f}')
        
#         fig.canvas.draw_idle()

# # Connect the click event
# fig.canvas.mpl_connect('button_press_event', on_click)

# plt.show()





# ############################### Plot cost evolution as table

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