import os
import csv
import numpy as np
import pandas as pd
import plotly
import plotly.express as px
import plotly.graph_objs as go
import open3d as o3d

import matplotlib.pyplot as plt
import matplotlib as mlib
from matplotlib import colors
from matplotlib import cm


##################################################################################################
## INPUT PARAMETERS

n_div = 2 # Grid division

#data_directory="/home/pal/Desktop/all/dataset/Picks/PCD/segmented/"
#data_directory="/home/pal/Desktop/more_data/dataset/PCD/"
#data_directory="/home/pal/Desktop/more_folds/dataset/PCD/"
data_directory="/home/pal/Desktop/all/PCD/"
#write_directory = "./new_results/filling/" #+ str(n_div) + "x" + str(n_div) + "/" ##./results/2x2/
write_directory = "./metrics/not_filling_peso/" + str(n_div) + "x" + str(n_div) + "/"


## Canonical object
all_files = True
pcd_file = "o5-04_gr_e04.pcd"
pcd_dir = data_directory+pcd_file
syn_can = True
can_pcd_file = 'o2_gr_e11.pcd'
can_pcd_dir = data_directory+can_pcd_file


save_def_metric = True
use_filling_def_metric = False
def_metric_file = str(n_div) + "x" + str(n_div) + ".csv" ## CSV file to save def metric
def_metric_dir = write_directory+def_metric_file
if(save_def_metric):
    my_file = open(def_metric_dir, "wb")
    wr = csv.writer(my_file, delimiter=",")

## Save results
show_plot = False
show_plot_metrics = True
print_info = False

save_plots_dir = write_directory + "/plots/"
save_plots = False

save_grid_sizes = False
grid_sizes_file = "grid_sizes.csv"
grid_sizes_dir = write_directory+grid_sizes_file
if(save_grid_sizes):
    my_file = open(grid_sizes_dir, "wb")
    grid_sizes_wr = csv.writer(my_file, delimiter=",")

activate_print = False


objects = ["o1", "o2", "o3", "o4", "o5", "o1-01","o1-04", "o2-01", "o2-04", "o2-07", "o2-10", "o5-01", "o5-04", "o5-07", "o5-10"]
#metrics_name = ["M1","M2","M3","M4", "M5","M6","M7","M8","M9","M10","M11","M12","M13","M14","M15","M16","M17","M18","M19","M20","M21","M22","M23","M24","M25"]
#classes = ["A","B","A","C","A","B","A","C","A","B","A","C","C","A","D","D","F","D","D","G","D","D","F","D"]
#classes = ["A","B","B","B","B","B","B","B","B","B","B","B","B","A","D","D","D","C","C","C","C","C","C","C","C","C","A","C","C","C","E","C","C","E","C","E","C","C","C","A","B","B","B","B","B","B","B","B","B","B","B","B","A","C","C","C","C","C","C","E","C","C","C","C","E"]

#classes = ["Z","A","A","B","B","B","B","C","C","C","A","A","B","Z","E","E","E","D","D","D","D","D","D","D","D","D","Z","D","D","D","E","D","D","E","D","E","D","D","D","Z","A","A","A","C","C","C","A","A","A","B","B","B","Z","D","D","D","D","D","D","E","D","D","D","D","E"]
#n_classes = [0,1,1,2,2,2,2,3,3,3,1,1,2,0,5,5,5,4,4,4,4,4,4,4,4,4,0,4,4,4,5,4,4,5,4,5,4,4,4,0,1,1,1,3,3,3,1,1,1,2,2,2,0,4,4,4,4,4,4,3,4,4,4,4,4]
#classes = ["Z","A","A","A","A","A","A","A","A","A","A","A","A","Z","C","C","C","B","B","B","B","B","B","B","B","B","Z","B","B","B","C","B","B","C","B","C","B","B","B","Z","A","A","A","A","A","A","A","A","A","A","A","A","Z","B","B","B","B","B","B","C","B","B","B","B","C"]
#n_classes = [0,1,1,1,1,1,1,1,1,1,1,1,1,0,3,3,3,2,2,2,2,2,2,2,2,2,0,2,2,2,3,2,2,3,2,3,2,2,2,0,1,1,1,1,1,1,1,1,1,1,1,1,0,2,2,2,2,2,2,3,2,2,2,2,3]
classes = ["A","A","A","A","A","A","A","A","A","A","A","A","C","C","C","B","B","B","B","B","B","C","C","C","B","B","B","C","B","B","C","B","C","B","B","B","A","A","A","A","A","A","A","A","A","A","A","A","B","B","B","B","B","B","C","B","B","B","B","C"]
n_classes = [0,0,0,0,0,0,0,0,0,0,0,0,2,2,2,1,1,1,1,1,1,2,2,2,1,1,1,2,1,1,2,1,2,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,2,1,1,1,1,2]

classes_f = ["B","B","B","B","B","B","B","B","B","D","D","D","B","B","B","B","A","B","B","B","B","D","D","D","B","B","B","A","A","A"]
n_classes_f = [1,1,1,1,1,1,1,1,1,3,3,3,1,1,1,1,0,1,1,1,1,3,3,3,1,1,1,0,0,0]

classes_all = ["B","B","B","B","B","B","A","A","A","A","A","A","A","A","A","A","A","A","B","B","B","D","D","D","B","B","B","B","A","B","C","C","C","B","B","B","B","B","B","C","C","C","B","B","B","C","B","B","C","B","C","B","B","B","A","A","A","A","A","A","A","A","A","A","A","A","B","B","B","D","D","D","B","B","B","A","A","A","B","B","B","B","B","B","C","B","B","B","B","C"]
n_classes_all = [1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,3,3,3,1,1,1,1,0,1,2,2,2,1,1,1,1,1,1,2,2,2,1,1,1,2,1,1,2,1,2,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,3,3,3,1,1,1,0,0,0,1,1,1,1,1,1,2,1,1,1,1,2]

#n_classes_all = [0,0,0,0,0,0,0,0,0,0,0,0,2,2,2,1,1,1,1,1,1,2,2,2,1,1,1,2,1,1,2,1,2,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,2,1,1,1,1,2,1,1,1,1,1,1,1,1,1,3,3,3,1,1,1,1,0,1,1,1,1,3,3,3,1,1,1,0,0,0]

classes = classes_all
n_classes = n_classes_all



n_exp=0

object = "o1"
# towel = True
# pillowc =  False
# checkered = False
# waffle = False
# linen = False

##################################################################################################
## UTIL FUNCTIONS

def print_info(activate, arg1, arg2="", arg3="", arg4="", arg5="", arg6=""):
    if(activate):
        #print(arg1)
        #print(arg2)
        print(str(arg1) + str(arg2) + str(arg3) + str(arg4) + str(arg5) + str(arg6))

def peso_def_metric(def_metric, dim):
    y = 5*dim #50
    def_metric = def_metric*y
    def_metric = def_metric/100000

    return def_metric

def plot(data, file_name):
    data = np.array(data)
    fig = px.scatter_3d(x=data[:,0], y=data[:,1], z=data[:,2], color=data[:,2])
    #plotly.offline.plot({"data": [fig1], "layout": mylayout}, auto_open=True)
    fig.show()
    filename = "./plots/" + file_name + ".jpg"
    #fig.write_image(filename)

## Saves RGB images with the corresponding filename, GT class and metrics
def plot_with_info(can, data, grids, x_grid_divs, y_grid_divs, can_mean_depth, def_measure, file_name):
    print("\033[94m Plotting with info... \033[0m")
    can = np.array(can)
    data = np.array(data)
    planes_x = []
    planes_y = []
    x_data=data[:,0]
    y_data=data[:,1]
    z_data=data[:,2]
    bright_blue = [[0, '#7DF9FF'], [1, '#7DF9FF']]
    bright_pink = [[0, '#FF007F'], [1, '#FF007F']]

    # Plot garment
    fig = px.scatter_3d(x=data[:,0], y=data[:,1], z=data[:,2], color=data[:,2])

    # Plot X axis divisions
    for n in range(1,len(y_grid_divs)-1):
        x=x_grid_divs[n]*np.ones(len(x_data))
        y=np.linspace(min(y_data),max(y_data),100)
        z=np.linspace(min(z_data)-0.01,max(z_data)+0.01,50)
        plane = go.Surface(x=x, y=y, z=np.array([z]*len(x)), colorscale=bright_blue, opacity=0.6)
        planes_x.append(plane)
    # Plot Y axis divisions
    for n in range(1,len(y_grid_divs)-1):
        x=np.linspace(min(x_data),max(x_data),100)
        y=y_grid_divs[n]*np.ones(len(y_data))
        z=np.linspace(min(z_data)-0.01,max(z_data)+0.01,50)
        plane = go.Surface(x=x, y=y, z=np.array([z]*len(x)).T, colorscale=bright_blue, opacity=0.6)
        planes_y.append(plane)

    # Plot grids mean values
    grids = np.array(grids)
    values = []
    for i in range(len(grids)):
        cent_x = np.sum(grids[i][:,0])/len(grids[i])
        cent_y = np.sum(grids[i][:,1])/len(grids[i])
        #value = [cent_x, cent_y, can_mean_depth+def_measure[i]]
        value = [cent_x, cent_y, def_measure[i]]
        values.append(value)
    #print("Values: ", values)
    values = np.array(values)
    point = go.Scatter3d(x=values[:,0], y=values[:,1], z=values[:,2], marker=dict(colorscale=bright_blue))

    can = go.Scatter3d(x=can[:,0], y=can[:,1], z=can[:,2])
    fig.add_traces(can)
    fig.add_traces(planes_x)
    fig.add_traces(planes_y)
    fig.add_traces(point)
    #fig.update_layout(scene=dict(zaxis=dict(range=[max(z_data), min(z_data)]), xaxis=dict(range=[max(x_data), min(x_data)])))
    fig.update_layout(scene=dict(zaxis=dict(range=[0.2, -0.2]), xaxis=dict(range=[0.3, -0.3]), yaxis=dict(range=[-0.2, 0.2]) ))
#    fig.update_layout(scene=dict(zaxis=dict(range=[0.31, 0])))
#    fig.write_image(file_name)
    #plotly.offline.plot({"data": [fig1], "layout": mylayout}, auto_open=True)
    fig.show()

def plot_metrics(filename, metrics):
    print("\033[94m Plotting deformation pattern images... \033[0m")

    data = np.array([0,1,2,3,4,5,6,7,8])
    #print(metrics)
    size = len(metrics)
    div=int(np.sqrt(size))
    new_row = []
    new_data = np.zeros([1,div])

    ## Sorter metrics in a matriz according to its corresponding grid position
    for n in range(0,div):
        for i in range(0+n,size,div):#np.sqrt(size)):#size[0],size[0]):
            new_row.append(metrics[i])
        new_data = np.vstack((new_data,new_row))
        new_row[:] = []
    data = np.delete(new_data,0,0) ## Delete first row (initialization of matrix)

    ## create discrete colormap
    # cmap = colors.ListedColormap(['red', 'blue'])
    # bounds = [0,0.02,0.05]
    # norm = colors.BoundaryNorm(bounds, cmap.N)

    fig, ax = plt.subplots()
    #ax.imshow(data, cmap='gist_rainbow', norm=colors.LogNorm(vmin=data.min(), vmax=0.1))#data.max()))
    #ax.imshow(data, cmap='gist_rainbow', norm=colors.LogNorm(vmin=data.min(), vmax=data.max()))

    # if use_filling_def_metric:
    #     print("hola")
    #     ax.imshow(data, cmap='gist_rainbow', norm=colors.LogNorm(vmin=0.00, vmax=0.28)) # 0.01-0.3 ## 0.001-0.5 (azul-rosa) / 0.01-0.5 (verde-azul-lila) / 0.01-1 (verde-azul) /
    # else:
    #     print("hola2")
    #     ax.imshow(data, cmap='gist_rainbow', norm=colors.LogNorm(vmin=data.min(), vmax=data.max()))

    #ax.imshow(data, cmap='gist_rainbow', norm=colors.LogNorm(vmin=data.min(), vmax=2))#data.max()))
#    ax.imshow(data, cmap=cmap, norm=norm)
#    ax.imshow(data, cmap=cm.coolwarm, norm=colors.LogNorm(vmin=0.0,vmax=0.1))

    # draw gridlines
    #ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=2)
    #ax.set_xticks(np.arange(0, 1, 1));
    #ax.set_yticks(np.arange(0, 1, 1));
    plt.axis("off")

    #ax.imshow(data, cmap='gist_rainbow', norm=colors.LogNorm(vmin=0, vmax=1))
    #ax.imshow(data, cmap='gist_rainbow', norm=colors.LogNorm(vmin=0.001, vmax=1.0)) #10x10
    ax.imshow(data, cmap='gist_rainbow', norm=colors.LogNorm(vmin=0.00002, vmax=0.02)) #2x2
    #plt.show()
    file_name = write_directory + filename + "_pat.png"
    plt.savefig(file_name)

## METRIC FUNCTIONS

def remove_gripper(obj_data):
    gripper_thrs = [0.05, 0.0, -0.15] #xmax, xmin, ymax

    rightx_data = obj_data[gripper_thrs[0]<obj_data[:,0]]
    leftx_data = obj_data[gripper_thrs[1]>obj_data[:,0]]

    boty_data = obj_data[gripper_thrs[2]<obj_data[:,1]]
    boty_data2 = boty_data[gripper_thrs[0]>boty_data[:,0]]
    boty_data3 = boty_data2[gripper_thrs[1]<boty_data2[:,0]]

    without_gripper = np.vstack((rightx_data,leftx_data))
    without_gripper = np.vstack((without_gripper,boty_data3))

    min_y_data = min(without_gripper[:,1])

    obj_data_cut_gripper = obj_data[min_y_data<obj_data[:,1]]

    #plot(without_gripper,"hola")
    #plot(obj_data_cut_gripper,"hola")
    #print("MIN withhout gripper", min_y_data)

    return min_y_data, obj_data_cut_gripper

def get_canonical(obj_name, align_canonical, n_div):
    ##With synthetic canonical data
    if(syn_can):
        can_data, can_edge_size, can_dim = create_canonical(obj_name, align_canonical)
        can_x_grid_divs, can_y_grid_divs, can_grids = can_grid_division(can_data, n_div) # Divide garment in grid
        can_min_depth, can_mean_depth, can_max_grid_len = canonical_params(can_data, can_grids) # Get canonical parameters
        can_def_measures, can_transl_data = deformation_metric(can_grids, can_min_depth, can_max_grid_len, can_edge_size, can_data, can_grids) # Compute deformation metrics (grids depth mean)
        #if(show_plot):
            #plot(can_transl_data, "transl_canonical")
            #plot(can_data, "CANONICAL")
            #plot_with_info(can_transl_data, can_transl_data, can_grids, can_x_grid_divs, can_y_grid_divs, can_min_depth, can_def_measures, "hola")
        # if(save_grid_sizes):
        #     write_grid_sizes("o1_gr_syn_can", can_grids)

    ## With REAL Canonical object
    if not syn_can:
        print("\033[96m Getting canonical file: \033[0m ", can_pcd_file)
        can_pcd = o3d.io.read_point_cloud(can_pcd_dir)
        can_data = np.asarray(can_pcd.points)
        print_info(activate_print, can_data.shape)
        can_x_grid_divs, can_y_grid_divs, can_grids = can_grid_division(can_data, n_div) # Divide garment in grid
        can_min_depth, can_mean_depth, can_max_grid_len = canonical_params(can_data, can_grids) # Get canonical parameters
        #Necessary to know canonical object edge size (manually or by detecting the corners)
        can_def_measures, can_transl_data = deformation_metric(can_grids, can_min_depth, can_max_grid_len, can_edge_size, can_data, can_grids) # Compute deformation metrics (grids depth mean)
        #plot(can_transl_data, "transl_canonical")
        #plot(can_data, "CANONICAL")

    return can_data, can_x_grid_divs, can_y_grid_divs, can_grids, can_min_depth, can_max_grid_len, can_def_measures, can_transl_data, can_edge_size, can_dim

def create_canonical(obj_name, align_canonical):
    print("\033[90m create_canonical \033[0m")
    #Extract object size from point cloud
    #Having px/cm, distance to camera and object size compute new size and number of points
    #Create a rectangle with depth mean and new size (X,Y) with center the camera

    #X, Y?
    syn_can_matrix = []
    syn_can_x = []
    syn_can_y = []
    syn_can_depth = []
    print_info(activate_print, obj_name)

    if(obj_name == "o1-01"): #can01
        print("\033[96m Creating canonical for o1-01... \033[0m")
        xsteps = 0.009
        xmin = -0.2
        xmax = xmin+0.46
        ysteps = 0.01
        ymin = -0.184
        ymax = ymin+0.26
        obj_edge_size = 0.25
    if(obj_name == "o1-04"): #can04
        print("\033[96m Creating canonical for o1-04... \033[0m")
        xsteps = 0.009
        xmin = -0.21
        xmax = xmin+0.46
        ysteps = 0.01
        ymin = -0.184
        ymax = ymin+0.19
        obj_edge_size = 0.25
    if(obj_name == "o2-01"):
        print("\033[96m Creating canonical for o2-01 \033[0m")
        xsteps = 0.009
        xmin = -0.19
        xmax = xmin+0.44
        ysteps = 0.01
        ymin = -0.196
        ymax = ymin+0.2
        obj_edge_size = 0.28 # 0.22?
    if(obj_name == "o2-04"):
        print("\033[96m Creating canonical for o2-04 \033[0m")
        xsteps = 0.009
        xmin = -0.09
        xmax = xmin+0.23#0.13
        ysteps = 0.01
        ymin = -0.196
        ymax = ymin+0.28#0.0
        obj_edge_size = 0.28
    if(obj_name == "o2-07"):
        print("\033[96m Creating canonical for o2-07 \033[0m")
        xsteps = 0.009
        xmin = -0.11
        xmax = xmin+0.28#0.16
        ysteps = 0.01
        ymin = -0.196
        ymax = ymin+0.16
        obj_edge_size = 0.15 #0.28
    if(obj_name == "o2-10"):
        print("\033[96m Creating canonical for o2-10 \033[0m")
        xsteps = 0.009
        xmin = -0.08
        xmax = xmin+0.23#0.16
        ysteps = 0.01
        ymin = -0.196
        ymax = ymin+0.15
        obj_edge_size = 0.12 #0.28
    if(obj_name == "o5-01"):
        print("\033[96m Creating canonical for o5-01 \033[0m")
        xsteps = 0.009
        xmin = -0.145
        xmax = xmin+0.35
        ysteps = 0.01
        ymin = -0.19
        ymax = ymin+0.18
        obj_edge_size = 0.18#0.24
    if(obj_name == "o5-04"):
        print("\033[96m Creating canonical for o5-04 \033[0m")
        xsteps = 0.009
        xmin = -0.06
        xmax = xmin+0.19
        ysteps = 0.01
        ymin = -0.196
        ymax = ymin+0.22#0.25
        obj_edge_size = 0.25
    if(obj_name == "o5-07"):
        print("\033[96m Creating canonical for o5-07 \033[0m")
        xsteps = 0.009
        xmin = -0.09
        xmax = xmin+0.24
        ysteps = 0.01
        ymin = -0.196
        ymax = ymin+0.13
        obj_edge_size = 0.12
    if(obj_name == "o5-10"):
        print("\033[96m Creating canonical for o5-10 \033[0m")
        xsteps = 0.009
        xmin = -0.07
        xmax = xmin+0.19
        ysteps = 0.01
        ymin = -0.196
        ymax = ymin+0.13
        obj_edge_size = 0.12

    if(obj_name == "o1"):
        print("\033[96m Creating canonical for o1... \033[0m")
        xsteps = 0.009
        xmin = -0.1
        xmax = xmin+0.24 #0.14
        ysteps = 0.01
        ymin = -0.184
        ymax = ymin+0.24 #0.06
        obj_edge_size = 0.25 #previous max_seen_depth = 0.16 (e11)
    if(obj_name == "o2"):
        print("\033[96m Creating canonical for o2 \033[0m")
        xsteps = 0.009
        xmin = -0.12
        xmax = 0.16 #xmin+0.24 #0.15
        ysteps = 0.01
        ymin = -0.196
        ymax = 0.0 #0.03 #ymin+0.24 #0.02
        obj_edge_size = 0.28 #previous max_seen_depth = 0.2 (e01)
    if(obj_name == "o3"):
        print("\033[96m Creating canonical for o3 \033[0m")
        xsteps = 0.009
        xmin = -0.12
        xmax = xmin+0.27
        ysteps = 0.01
        ymin = -0.19
        ymax = ymin+0.17 #+0.19
        obj_edge_size = 0.24 #size:0.26 #previous max_seen_depth = 0.15 (e04)
    if(obj_name == "o4"):
        print("\033[96m Creating canonical for o4 \033[0m")
        xsteps = 0.009
        xmin = -0.12
        xmax = xmin+0.27
        ysteps = 0.01
        ymin = -0.19
        ymax = ymin+0.19
        obj_edge_size = 0.16 #size: 0.26 #previous max_seen_depth = 0.12 (e05)
    if(obj_name == "o5"):
        print("\033[96m Creating canonical for o5 \033[0m")
        xsteps = 0.009
        xmin = -0.11
        xmax = xmin+0.25
        ysteps = 0.01
        ymin = -0.19
        ymax = ymin+0.17#0.18
        obj_edge_size = 0.24 #previous max_seen_depth = 0.17 (e07)

    ## Align syn canonical to cloth object without gripper
    # if(align_canonical>ymin):
    #     print("align > ymin")
    #     dif_alignment = align_canonical-ymin
    #     ymin = ymin+dif_alignment
    #     ymax = ymax + dif_alignment
    # else:
    #     dif_alignment = ymin-align_canonical
    #     ymin = ymin + dif_alignment
    #     ymax = ymax + dif_alignment
    dif_alignment = align_canonical-ymin
    ymin = ymin+dif_alignment
    ymax = ymax + dif_alignment

    if(print_info):
        print_info(activate_print, "Dif alignment canonical: ", dif_alignment)
        print_info(activate_print, "Xmin: ", xmin, "Xmax: ", xmax)
        print_info(activate_print, "New ymin: ", ymin, "New ymax: ", ymax)

    syn_can_x = np.arange(xmin,xmax,xsteps)
    syn_can_y = np.arange(ymin,ymax,ysteps)
    print("X: ", len(syn_can_x))
    print("Y: ", len(syn_can_y))

    syn_can_matrix=[syn_can_x[0],syn_can_y[0],0.42]
    for i in range(0,len(syn_can_x)):
        for n in range(0,len(syn_can_y)):
            row = [syn_can_x[i],syn_can_y[n],0.42]
            syn_can_matrix = np.vstack([syn_can_matrix, row])

    #print_info(True, "Xmin: ", xmin, "Xmax: ", xmax)
    #print_info(True, "New ymin: ", ymin, "New ymax: ", ymax)
    print("Size: X: ", xmax-xmin, " / Y: ", ymax-ymin)
    print_info(True, len(syn_can_matrix))

    ## Canonical area
    can_dim = ((xmax-xmin)*100)*((ymax-ymin)*100)
    print_info(True, can_dim)

    return syn_can_matrix, obj_edge_size, can_dim

def can_grid_division(data, n_div):
    print("\033[96m Dividing in grids... \033[0m")
    x_thrs = []
    y_thrs = []
    grids = []
    x = data[:,0]
    y = data[:,1]

    min_x = min(data[:,0])
    max_x = max(data[:,0])
    min_y = min(data[:,1])
    max_y = max(data[:,1])
    # if(pillowc):
    #     max_y = max(data[:,1])#0.05)
    min_z = min(data[:,2])
    max_z = max(data[:,2])
    x_thr = (max_x - min_x)/n_div
    y_thr = (max_y - min_y)/n_div
    z_thr = (max_z - min_z)/n_div

    ## Grids
    ## Get XY thresholds based on given number divisions
    x_thrs.append(min_x-1)
    y_thrs.append(min_y-1)
    for n in range(1,n_div):
        next_x_thr = min_x + (x_thr*n)
        x_thrs.append(next_x_thr)
        next_y_thr = min_y + (y_thr*n)
        y_thrs.append(next_y_thr)
    x_thrs.append(max_x+1)
    y_thrs.append(max_y+1)
    if(print_info):
        print_info(activate_print, "X threshohld: ", x_thrs, " / Y threshohld: ", y_thrs)

    ## Cluster different grids
    for n in range(n_div):
        grid = data[x_thrs[n]<=data[:,0]]
        grid = grid[x_thrs[n+1]>=grid[:,0]]
        for b in range(n_div):
            gridy = grid[y_thrs[b]<=grid[:,1]]
            grid2 = gridy[y_thrs[b+1]>gridy[:,1]]
            #file_n = str(n)+str(b)+".html"
            #plot(grid2,file_n)
            grids.append(grid2)

#    grids_complete = complete_canonical(grids)
    print_info(activate_print, "Canonical data size: ", len(data))
    print_info(activate_print, "Grid1 size: ", len(grids[0]))
    print_info(activate_print, "Grid2 size: ", len(grids[1]))
    print_info(activate_print, "Grid3 size: ", len(grids[2]))
    print_info(activate_print, "Grid4 size: ", len(grids[3]))

    return x_thrs, y_thrs, grids

def grid_division(data, x_thrs, y_thrs, n_div):
    print("\033[96m Dividing in grids... \033[0m")
    grids = []

    ## Cluster different grids
    for n in range(n_div):
        grid = data[x_thrs[n]<=data[:,0]]
        grid = grid[x_thrs[n+1]>grid[:,0]]
        for b in range(n_div):
            gridy = grid[y_thrs[b]<=grid[:,1]]
            grid2 = gridy[y_thrs[b+1]>gridy[:,1]]
            #file_n = str(n)+str(b)+".html"
            #plot(grid2,file_n)
            grids.append(grid2)

    print_info(activate_print,"Data size: ", len(data))
    for i in range(0,len(grids)):
        print_info(activate_print,"Grid size: ", len(grids[i]))
        print_info(activate_print,"Data size: ", len(data))
        #print("Sum grid sizes: ", len(grids[0])+len(grids[1])+len(grids[2])+len(grids[3]))
        for i in range(0,len(grids)):
            print_info(activate_print,"Grid size: ", len(grids[i]))

    return grids

def canonical_params(data, grids):
    # Get canonical grid sizes
    grid_sizes = []
    max_can_grid_len = 0

    ## Get max grid size
    for i in range(len(grids)):
        grid_sizes.append(len(grids[i]))
        can_grid_len = len(grids[i])
        if(can_grid_len > max_can_grid_len):
            max_can_grid_len = can_grid_len

    depth = data[:,2]
# Get mean depth (or min?)
    mean_depth = np.mean(depth)
    min_depth = min(depth)

    print_info(activate_print,"Can max grid len: ", max_can_grid_len)
    if(print_info):
        print_info(activate_print,"Max grid len: ", max_can_grid_len)
        print_info(activate_print,"Canonical mean: ", mean_depth)
        print_info(activate_print,"Canonical min: ", min_depth)

    return min_depth, mean_depth, max_can_grid_len

def deformation_metric(can_grids, can_min_depth, can_max_grid_len, can_edge_size, obj_data, grids):
    print("\033[94m Computing deformation metric... \033[0m")
    means = []
    suma = 0
    transl_data = []
    obj_depth = obj_data[:,2]
    min_depth=min(obj_depth)
    max_depth=max(obj_depth)
    if(max_depth<0.7):
        max_depth_trasl = max_depth-min_depth #can_min_depth
    else:
        print_info(activate_print,"max depth > 1: ")
        max_depth_trasl = 0.62-min_depth # can_min_depth # 0.62 should be max posible depth (half size cloth)
    print_info(activate_print, "Max depth: ", max_depth, " - Min depth: ", min_depth, " = ", max_depth_trasl)

    for l in range (len(grids)):
        #mean1 = np.mean(grids[l][:,2])
        #mean1 = mean1-0.42
        fill = False
        x = grids[l][:,0]
        y = grids[l][:,1]
        can_x = can_grids[l][:,0]
        can_y = can_grids[l][:,1]

        depth = grids[l][:,2]
        new_grid = grids[l]
        ## Depth points traslation
        suma = 0
        for i in range(len(depth)):
            #point = (depth[i]-can_min_depth)/(1-can_min_depth)
            point = depth[i] - min_depth #can_min_depth #0.42
            suma += point
            new_point=[new_grid[i][0], new_grid[i][1], point]
            transl_data.append(new_point)

        print_info(activate_print, "sum visible depths: ", suma)

    ## Check if it is necessary to fill
        fill=True
        # max_x_grid = max(x)
        # min_x_grid = min(x)
        # mean_x_grid = np.mean(x)
        # #print(mean_x_grid)
        # print("Min: ", min_x_grid, " / Min can: ", min(can_x))
        # print("Max: ", max_x_grid, " / Max can: ", max(can_x))
        # if(max_x_grid < (max(can_x)-0.025) or min_x_grid > (min(can_x)+0.055)):
        #     print("Can  be filled")
        #     fill = True

        if(fill):
            ## Fill empty points
            if len(grids[l]) < can_max_grid_len: #(can_grids[l]):
                #length = len(can_grids[l])
                length = can_max_grid_len
            #if len(grids[l]) < length:
                #print("Filling empty points! l=",l)
                dif = length-len(grids[l])
                #dif = length-len(grids[l])
                suma += dif*can_edge_size #max_depth_trasl
                #print("FILLING")
                print_info(activate_print, "Dif length: ", dif, " (", l,")")
            else:
                #length = len(can_grids[l])
                length = len(grids[l])
        else:
            length = len(grids[l])
        ## Grid depth mean
        #print("SUMA / Length: ", suma, " / ", length)
        dif_to_mean = suma/length
        means.append(dif_to_mean)

    print_info(activate_print,"Means: ", means)

    return means, transl_data

def new_def_metric(grids, can_dim):
    print("\033[94m Computing deformation metric NOT FILLING... \033[0m")
    ## Compute deformation metric where it is not necessary to fill the occluded points
    def_metrics = []
    transl_data = []

    obj_depth = obj_data[:,2]
    min_depth=min(obj_depth)

    ## For each section of the grid
    for l in range (len(grids)):
        suma = 0
        length = len(grids[l])
        print_info(activate_print, "\033[94m length \033[0m", length)
        depth = grids[l][:,2]
        new_grid = grids[l]

        for i in range(len(depth)):
            point = depth[i] - min_depth ## Depth points traslation
            suma += point                ## Compute mean of current grid
            new_point=[new_grid[i][0], new_grid[i][1], point]
            transl_data.append(new_point) ## Create new traslated grid
        print_info(activate_print, "sum visible depths: ", suma)

        ## When the grid is empty -> set def metric to 1
        if(length == 0): #<= 1):
            print_info(activate_print, "Empty grid!")
            mean = 1
        ## When there are visible points compute mean and then divide again by grid size => def metric
        else:
            mean = suma/length
            mean = mean/length
            mean=mean*100
            ## Pesar deformation metric segun tamano
            mean = peso_def_metric(mean, can_dim)

        ## When there are few visible points the mean will be > 1. Then set def metric to max (1)
        # if(mean>1):
        #     print_info(activate_print, "Mean > 1")
        #     mean = 1
        def_metrics.append(mean)

    print_info(activate_print,"Means: ", def_metrics)

    # #Compute deformation measure
    #     if len(grids[l]) < can_max_grid_len:
    #         length = can_max_grid_len
    #         dif = length-len(grids[l])
    #         suma += dif*can_edge_size #max_depth_trasl
    #         print("Dif length: ", dif, " (", l,")")
    #     else:
    #         #length = len(can_grids[l])
    #         length = len(grids[l])



    return def_metrics, transl_data

def get_resolution(obj_data):

    pixel_thrs = [0.02, 0.03, -0.12, -0.11]

    data = obj_data[pixel_thrs[0]<=obj_data[:,0]]
    data2 = data[pixel_thrs[1]>=data[:,0]]
    data3 = data2[pixel_thrs[2]<=data2[:,1]]
    px_square = data3[pixel_thrs[3]>=data3[:,1]]

    resolution = len(px_square)
    print("Resolution: ", resolution) ## Number of point in 0.01

    return resolution

def save_mean_values(exp_name, n_exp, mean_data):
    print("\033[94m Writing deformation metric values... \033[0m")
    data = []

    #print(mean_data)
    data.append(exp_name)
    data.append(classes[n_exp])
    data.append(n_classes[n_exp])
    for i in range(len(mean_data)):
        data.append(mean_data[i])
    wr.writerow(data)

def write_grid_sizes(exp_name, obj_grids):
    print("\033[94m Writing grid sizes... \033[0m")
    data_size = []
    data_size.append(exp_name)
    for i in range(len(obj_grids)):
        data_size.append(len(obj_grids[i]))
    grid_sizes_wr.writerow(data_size)

def write_csv(csv_file, exp_name, data):
    my_file = open(csv_file, "wb")
    wr = csv.writer(my_file, delimiter=",")
    data_size = []

    data_size.append(exp_name)
    for i in range(len(data)):
        data_size.append(data[i])
    wr.writerow(data_size)



##################################################################################################
## MAIN
#
# test = np.arange(0,10,0.02)
# print(test)
# print(len(test))
# syn_can_x = np.arange(ymin,ymax,xsteps)

######## For one unique experiment
if not all_files :
    print("\033[94m Getting experiment file: ", pcd_file, "\033[0m")
    for n in range(len(objects)):
        if objects[n] in pcd_file:
            obj_name = objects[n]
    obj_pcd = o3d.io.read_point_cloud(pcd_dir) #Read pcd files from folder
    obj_data = np.asarray(obj_pcd.points)
    plot(obj_data, "EXPERIMENT")
    ## Remove gripper and get minimum x of object to align canonical
    align_canonical, obj_data = remove_gripper(obj_data)
    ## Get canonical
    can_data, can_x_grid_divs, can_y_grid_divs, can_grids, can_min_depth, can_max_grid_len, can_def_measures, can_transl_data, can_edge_size = get_canonical(obj_name, align_canonical, n_div)
    #get_resolution(can_data)
    ## Divide grids
    obj_grids = grid_division(obj_data, can_x_grid_divs, can_y_grid_divs, n_div) #Divide grids
    ## Compute deformation metrics (grids depth mean)
    if use_filling_def_metric:
        def_measures, obj_transl_data = deformation_metric(can_grids, can_min_depth, can_max_grid_len, can_edge_size, obj_data, obj_grids) # Compute deformation metrics (grids depth mean)
    else:
        def_measures, obj_transl_data = new_def_metric(obj_grids)
    plot(obj_transl_data, "transl exp")
    ## Save results
    if(save_def_metric): ##Save means in csv
        ##Write CSV headers
        headers = ["File"]
        for i in range(0, n_div*n_div):
            text = "M"+str(i+1)
            headers.append(text)
        #wr.writerow(headers)
        #save_mean_values(pcd_file.replace("_seg.pcd", ""), n_exp, def_measures)
    if(save_grid_sizes): ##Save grid sizes
        write_grid_sizes(pcd_file.replace(".pcd", ""), obj_grids)
    if(show_plot):
        plotname = save_plots_dir + pcd_file.replace(".pcd", "_plot.png")
        plot_with_info(can_transl_data, obj_transl_data, obj_grids, can_x_grid_divs, can_y_grid_divs, can_min_depth, def_measures, plotname)
    if(show_plot_metrics):
        plot_metrics(filename.replace(".pcd", ""), def_measures)


######### For all experiments #########
if(all_files):
    ##Read pcd files from folder
    print("\033[94m Reading PCD files \033[0m")
    print(data_directory)
    ##Write CSV headers
    if(save_def_metric):
        headers = ["File","Class_GT","Class_GT_n"]
        #headers = ["File"]
        for i in range(0, n_div*n_div):
            #headers.append(metrics_name[i])
            text = "M"+str(i+1)
            headers.append(text)
        wr.writerow(headers)
    for filename in sorted(os.listdir(data_directory)):
        f = os.path.join(data_directory, filename)
        if os.path.isfile(f) and filename.endswith('.pcd'):
            print("-------------------------------------------------------------------------------")
            for n in range(len(objects)):
                if objects[n] in filename:
                    obj_name = objects[n]
            print(filename)
            obj_pcd = o3d.io.read_point_cloud(f)
            obj_data = np.asarray(obj_pcd.points)
            ## Remove gripper and get minimum x of object to align canonical
            align_canonical, obj_data = remove_gripper(obj_data)
            ## Get canonical
            can_data, can_x_grid_divs, can_y_grid_divs, can_grids, can_min_depth, can_max_grid_len, can_def_measures, can_transl_data, can_edge_size, can_dim = get_canonical(obj_name, align_canonical, n_div)
            #get_resolution(can_data)
            ##Divide grids
            obj_grids = grid_division(obj_data, can_x_grid_divs, can_y_grid_divs, n_div) #Divide grids
            ## Compute deformation metrics (grids depth mean)
            if use_filling_def_metric:
                def_measures, obj_transl_data = deformation_metric(can_grids, can_min_depth, can_max_grid_len, can_edge_size, obj_data, obj_grids)
            else:
                def_measures, obj_transl_data = new_def_metric(obj_grids, can_dim)
            if(save_def_metric): ##Save means in csv
                save_mean_values(filename.replace(".pcd", ""), n_exp, def_measures)
                n_exp+=1
            if(save_grid_sizes): ##Save grid sizes
                write_grid_sizes(filename.replace(".pcd", ""), obj_grids)
            if(show_plot):
                plotname = save_plots_dir + filename.replace(".pcd", ".jpg")
                plot_with_info(can_transl_data, obj_transl_data, obj_grids, can_x_grid_divs, can_y_grid_divs, can_min_depth, def_measures, plotname)
            if(show_plot_metrics):
                plot_metrics(filename.replace(".pcd", ""), def_measures)




#### REFS
# https://stackoverflow.com/questions/69372448/plotly-vertical-3d-surface-plot-in-z-x-plane-not-showing-up
# https://stackoverflow.com/questions/62403763/how-to-add-planes-in-a-3d-scatter-plot
# https://stackoverflow.com/questions/43971138/python-plotting-colored-grid-based-on-values
# # https://matplotlib.org/stable/tutorials/colors/colormaps.html
