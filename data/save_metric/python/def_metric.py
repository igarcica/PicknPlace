import os
import csv
import numpy as np
import pandas as pd
import plotly
import plotly.express as px
import plotly.graph_objs as go
import open3d as o3d

import matplotlib.pyplot as plt

n_div = 5 # Grid division

#data_directory="/home/pal/Desktop/all/dataset/Picks/PCD/segmented/"
data_directory="/home/pal/Desktop/more_data/dataset/PCD/"
write_directory = "./results/6/" + str(n_div) + "x" + str(n_div) + "/" ##./results/2x2/


## Canonical object
all_files = True
pcd_file = "o2_gr_e01.pcd"
pcd_dir = data_directory+pcd_file
syn_can = True
can_pcd_file = 'o3_gr_can.pcd'
can_pcd_dir = data_directory+can_pcd_file

## CSV file to save def metric
save_def_metric = True
def_metric_file = str(n_div) + "x" + str(n_div) + ".csv" ##o1_2x2.csv
def_metric_dir = write_directory+def_metric_file
if(save_def_metric):
    my_file = open(def_metric_dir, "wb")
    wr = csv.writer(my_file, delimiter=",")

## Save results
show_plot = False
print_info = False

save_plots_dir = write_directory + "/plots/"
save_plots = False

save_grid_sizes = False
grid_sizes_file = "grid_sizes.csv"
grid_sizes_dir = write_directory+grid_sizes_file
if(save_grid_sizes):
    my_file = open(grid_sizes_dir, "wb")
    grid_sizes_wr = csv.writer(my_file, delimiter=",")

objects = ["o1", "o2", "o3", "o4", "o5"]
metrics_name = ["M1","M2","M3","M4", "M5","M6","M7","M8","M9","M10","M11","M12","M13","M14","M15","M16","M17","M18","M19","M20","M21","M22","M23","M24","M25"]
#classes = ["A","B","A","C","A","B","A","C","A","B","A","C","C","A","D","D","F","D","D","G","D","D","F","D"]
#classes = ["A","B","B","B","B","B","B","B","B","B","B","B","B","A","D","D","D","C","C","C","C","C","C","C","C","C","A","C","C","C","E","C","C","E","C","E","C","C","C","A","B","B","B","B","B","B","B","B","B","B","B","B","A","C","C","C","C","C","C","E","C","C","C","C","E"]
classes = ["Z","A","A","B","B","B","B","C","C","C","A","A","B","Z","E","E","E","D","D","D","D","D","D","D","D","D","Z","D","D","D","E","D","D","E","D","E","D","D","D","Z","A","A","A","C","C","C","A","A","A","B","B","B","Z","D","D","D","D","D","D","E","D","D","D","D","E"]

n_exp=0

# if(all_files):
#     save_def_metric=True
#     show_plot = False
# else:
#     save_def_metric = False
#     show_plot = True

object = "o1"
# towel = True
# pillowc =  False
# checkered = False
# waffle = False
# linen = False

##################################################################################################

def plot(data, file_name):
    data = np.array(data)
    fig = px.scatter_3d(x=data[:,0], y=data[:,1], z=data[:,2], color=data[:,2])
    #plotly.offline.plot({"data": [fig1], "layout": mylayout}, auto_open=True)
    fig.show()
    filename = "./plots/" + file_name + ".jpg"
    #fig.write_image(filename)

def plot_with_info(can, data, grids, x_grid_divs, y_grid_divs, can_mean_depth, def_measure, file_name):
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
    fig.update_layout(scene=dict(zaxis=dict(range=[0.2, -0.2]), xaxis=dict(range=[0.2, -0.2]), yaxis=dict(range=[-0.2, 0.2]) ))
#    fig.update_layout(scene=dict(zaxis=dict(range=[0.31, 0])))
#    fig.write_image(file_name)
    #plotly.offline.plot({"data": [fig1], "layout": mylayout}, auto_open=True)
    fig.show()

def remove_gripper(obj_data):
    gripper_thrs = [0.03, 0.0, -0.15] #xmax, xmin, ymax

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
        can_data, can_edge_size = create_canonical(obj_name, align_canonical)
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
        print(can_data.shape)
        can_x_grid_divs, can_y_grid_divs, can_grids = can_grid_division(can_data, n_div) # Divide garment in grid
        can_min_depth, can_mean_depth, can_max_grid_len = canonical_params(can_data, can_grids) # Get canonical parameters
        #Necessary to know canonical object edge size (manually or by detecting the corners)
        can_def_measures, can_transl_data = deformation_metric(can_grids, can_min_depth, can_max_grid_len, can_edge_size, can_data, can_grids) # Compute deformation metrics (grids depth mean)
        #plot(can_transl_data, "transl_canonical")
        #plot(can_data, "CANONICAL")

    return can_data, can_x_grid_divs, can_y_grid_divs, can_grids, can_min_depth, can_max_grid_len, can_def_measures, can_transl_data, can_edge_size

def create_canonical(obj_name, align_canonical):
    #Extract object size from point cloud
    #Having px/cm, distance to camera and object size compute new size and number of points
    #Create a rectangle with depth mean and new size (X,Y) with center the camera

    #X, Y?
    syn_can_matrix = []
    syn_can_x = []
    syn_can_y = []
    syn_can_depth = []

    # if(towel):
    #     xsteps = 0.007
    #     xmin = -0.1
    #     xmax = 0.134
    #     ysteps = 0.007
    #     ymin = -0.184
    #     ymax = 0.04
    # if(pillowc):
    #     xsteps = 0.007
    #     xmin = -0.11
    #     xmax = 0.134
    #     ysteps = 0.007
    #     ymin = -0.196
    #     ymax = 0.02

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
        print("Dif alignment canonical: ", dif_alignment)
        print("New ymin: ", ymin, "New ymax: ", ymax)

    syn_can_x = np.arange(xmin,xmax,xsteps)
    syn_can_y = np.arange(ymin,ymax,ysteps)

    syn_can_matrix=[syn_can_x[0],syn_can_y[0],0.42]
    for i in range(0,len(syn_can_x)):
        for n in range(0,len(syn_can_y)):
            row = [syn_can_x[i],syn_can_y[n],0.42]
            syn_can_matrix = np.vstack([syn_can_matrix, row])

    return syn_can_matrix, obj_edge_size

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
        print("X threshohld: ", x_thrs, " / Y threshohld: ", y_thrs)

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
    print("Canonical data size: ", len(data))
    print("Grid1 size: ", len(grids[0]))
    print("Grid2 size: ", len(grids[1]))
    print("Grid3 size: ", len(grids[2]))
    print("Grid4 size: ", len(grids[3]))
    if(print_info):
        print("Data size: ", len(data))
       # print("Sum grid sizes: ", len(grids[0])+len(grids[1])+len(grids[2])+len(grids[3]))
       # print("Grid1 size: ", len(grids[3]))
       # print("Grid2 size: ", len(grids[1]))
       # print("Grid3 size: ", len(grids[2]))
       # print("Grid4 size: ", len(grids[0]))

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

    print("Data size: ", len(data))
    for i in range(0,len(grids)):
        print("Grid size: ", len(grids[i]))
    if(print_info):
        print("Data size: ", len(data))
        #print("Sum grid sizes: ", len(grids[0])+len(grids[1])+len(grids[2])+len(grids[3]))
        for i in range(0,len(grids)):
            print("Grid size: ", len(grids[i]))

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

    print("Can max grid len: ", max_can_grid_len)
    if(print_info):
        print("Max grid len: ", max_can_grid_len)
        print("Canonical mean: ", mean_depth)
        print("Canonical min: ", min_depth)

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
        print("max depth > 1: ")
        max_depth_trasl = 0.62-min_depth # can_min_depth # 0.62 should be max posible depth (half size cloth)
    print("Max depth: ", max_depth, " - Min depth: ", min_depth, " = ", max_depth_trasl)

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

        print("sum visible depths: ", suma)

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
                print("Dif length: ", dif, " (", l,")")
            else:
                #length = len(can_grids[l])
                length = len(grids[l])
        else:
            length = len(grids[l])
        ## Grid depth mean
        #print("SUMA / Length: ", suma, " / ", length)
        dif_to_mean = suma/length
        means.append(dif_to_mean)

    print("Means: ", means)
    if(print_info):
        print("Means: ", means)

    return means, transl_data

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
    #print("Writing mean values...")
    data = []

    #print(mean_data)
    data.append(exp_name)
    data.append(classes[n_exp])
    for i in range(len(mean_data)):
        data.append(mean_data[i])
    wr.writerow(data)

def write_grid_sizes(exp_name, obj_grids):
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

##With synthetic canonical data
# if(syn_can):
#     print("Getting synthetic canonical")
#     can_data = create_canonical(align_canonical)
#     can_x_grid_divs, can_y_grid_divs, can_grids = can_grid_division(can_data, n_div) # Divide garment in grid
#     can_min_depth, can_mean_depth, can_max_grid_len = canonical_params(can_data, can_grids) # Get canonical parameters
#     can_def_measures, can_transl_data = deformation_metric(can_grids, can_min_depth, can_max_grid_len, can_data, can_grids) # Compute deformation metrics (grids depth mean)
#     #if(show_plot):
#         #plot(can_transl_data, "transl_canonical")
#         #plot(can_data, "CANONICAL")
#         #plot_with_info(can_transl_data, can_transl_data, can_grids, can_x_grid_divs, can_y_grid_divs, can_min_depth, can_def_measures, "hola")
#     if(save_grid_sizes):
#         write_grid_sizes("o1_gr_syn_can", can_grids)
#
# ## With REAL Canonical object
# if not syn_can:
#     print("Getting canonical file: ", can_pcd_file)
#     can_pcd = o3d.io.read_point_cloud(can_pcd_dir)
#     can_data = np.asarray(can_pcd.points)
#     print(can_data.shape)
#     can_x_grid_divs, can_y_grid_divs, can_grids = can_grid_division(can_data, n_div) # Divide garment in grid
#     can_min_depth, can_mean_depth, can_max_grid_len = canonical_params(can_data, can_grids) # Get canonical parameters
#     can_def_measures, can_transl_data = deformation_metric(can_grids, can_min_depth, can_max_grid_len, can_data, can_grids) # Compute deformation metrics (grids depth mean)
#     #plot(can_transl_data, "transl_canonical")
#     #plot(can_data, "CANONICAL")


######## For one unique experiment
if not all_files :
    print("Getting experiment file: ", pcd_file)
    for n in range(len(objects)):
        if objects[n] in pcd_file:
            obj_name = objects[n]
    obj_pcd = o3d.io.read_point_cloud(pcd_dir) #Read pcd files from folder
    obj_data = np.asarray(obj_pcd.points)
    #plot(obj_data, "EXPERIMENT")
    ## Remove gripper and get minimum x of object to align canonical
    align_canonical, obj_data = remove_gripper(obj_data)
    ## Get canonical
    can_data, can_x_grid_divs, can_y_grid_divs, can_grids, can_min_depth, can_max_grid_len, can_def_measures, can_transl_data, can_edge_size = get_canonical(obj_name, align_canonical, n_div)
    #get_resolution(can_data)
    ## Divide grids
    obj_grids = grid_division(obj_data, can_x_grid_divs, can_y_grid_divs, n_div) #Divide grids
    ## Compute deformation metrics (grids depth mean)
    def_measures, obj_transl_data = deformation_metric(can_grids, can_min_depth, can_max_grid_len, can_edge_size, obj_data, obj_grids) # Compute deformation metrics (grids depth mean)
    #plot(obj_transl_data, "transl exp")
    ## Save results
    if(save_def_metric): ##Save means in csv
        ##Write CSV headers
        headers = ["File"]
        for i in range(0, n_div*n_div):
            headers.append(metrics_name[i])
        #wr.writerow(headers)
        #save_mean_values(pcd_file.replace("_seg.pcd", ""), n_exp, def_measures)
    if(save_grid_sizes): ##Save grid sizes
        write_grid_sizes(pcd_file.replace(".pcd", ""), obj_grids)
    if(show_plot):
        plotname = save_plots_dir + pcd_file.replace(".pcd", "_plot.png")
        plot_with_info(can_transl_data, obj_transl_data, obj_grids, can_x_grid_divs, can_y_grid_divs, can_min_depth, def_measures, plotname)


    ######### For all experiments
if(all_files):
    ##Read pcd files from folder
    print("Reading PCD files")
    print(data_directory)
    ##Write CSV headers
    if(save_def_metric):
        headers = ["File","Class_GT"]
        #headers = ["File"]
        for i in range(0, n_div*n_div):
            headers.append(metrics_name[i])
        wr.writerow(headers)
    for filename in sorted(os.listdir(data_directory)):
        f = os.path.join(data_directory, filename)
        if os.path.isfile(f) and filename.endswith('.pcd'):
            print("-------------------------------------------------------------------------------")
            for n in range(len(objects)):
                if objects[n] in filename:
                    obj_name = objects[n]
            print(f)
            obj_pcd = o3d.io.read_point_cloud(f)
            obj_data = np.asarray(obj_pcd.points)
            ## Remove gripper and get minimum x of object to align canonical
            align_canonical, obj_data = remove_gripper(obj_data)
            ## Get canonical
            can_data, can_x_grid_divs, can_y_grid_divs, can_grids, can_min_depth, can_max_grid_len, can_def_measures, can_transl_data, can_edge_size = get_canonical(obj_name, align_canonical, n_div)
            #get_resolution(can_data)
            ##Divide grids
            obj_grids = grid_division(obj_data, can_x_grid_divs, can_y_grid_divs, n_div) #Divide grids
            ## Compute deformation metrics (grids depth mean)
            def_measures, obj_transl_data = deformation_metric(can_grids, can_min_depth, can_max_grid_len, can_edge_size, obj_data, obj_grids)
            if(save_def_metric): ##Save means in csv
                save_mean_values(filename.replace(".pcd", ""), n_exp, def_measures)
                n_exp+=1
            if(save_grid_sizes): ##Save grid sizes
                write_grid_sizes(filename.replace(".pcd", ""), obj_grids)
            if(show_plot):
                plotname = save_plots_dir + filename.replace(".pcd", ".jpg")
                plot_with_info(can_transl_data, obj_transl_data, obj_grids, can_x_grid_divs, can_y_grid_divs, can_min_depth, def_measures, plotname)




#### REFS
# https://stackoverflow.com/questions/69372448/plotly-vertical-3d-surface-plot-in-z-x-plane-not-showing-up
# https://stackoverflow.com/questions/62403763/how-to-add-planes-in-a-3d-scatter-plot
