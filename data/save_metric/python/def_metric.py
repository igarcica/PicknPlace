import os
import csv
import numpy as np
import pandas as pd
import plotly
import plotly.express as px
import plotly.graph_objs as go
import open3d as o3d

import matplotlib.pyplot as plt

n_div = 2 # Grid division

data_directory="/home/pal/Desktop/all/dataset/Picks/PCD/segmented/"
write_directory = "./results/" + str(n_div) + "x" + str(n_div) + "/" ##./results/2x2/

## Canonical object
#can_obj_file = 'o1_gr_can_seg.pcd'
all_files = False
pcd_file = "o2_gr_e01_seg.pcd"
pcd_dir = data_directory+pcd_file

## CSV file to save def metric
save_def_metric = False
def_metric_file = str(n_div) + "x" + str(n_div) + ".csv" ##o1_2x2.csv
def_metric_dir = write_directory+def_metric_file
if(save_def_metric):
    my_file = open(def_metric_dir, "wb")
    wr = csv.writer(my_file, delimiter=",")

## Save results
show_plot = True
print_info = False

save_plots_dir = write_directory + "/plots/"
save_plots = False

save_grid_sizes = False
grid_sizes_file = "grid_sizes.csv"
grid_sizes_dir = write_directory+grid_sizes_file
if(save_grid_sizes):
    my_file = open(grid_sizes_dir, "wb")
    grid_sizes_wr = csv.writer(my_file, delimiter=",")

metrics_name = ["M1","M2","M3","M4", "M5","M6","M7","M8","M9","M10","M11","M12","M13","M14","M15","M16","M17","M18","M19","M20","M21","M22","M23","M24","M25"]
classes = ["A","B","A","C","A","B","A","C","A","B","A","C","C","A","D","D","F","D","D","G","D","D","F","D"]
n_exp=0


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
    #fig.update_layout(scene=dict(zaxis=dict(range=[max(z_data), min(z_data)])))
#    fig.update_layout(scene=dict(zaxis=dict(range=[0.31, 0])))
#    fig.write_image(file_name)
    #plotly.offline.plot({"data": [fig1], "layout": mylayout}, auto_open=True)
    fig.show()

def create_canonical():
    #Extract object size from point cloud
    #Having px/cm, distance to camera and object size compute new size and number of points
    #Create a rectangle with depth mean and new size (X,Y) with center the camera

    #X, Y?
    print("Creating synthetic canonical...")
    syn_can_matrix = []
    syn_can_x = []
    syn_can_y = []
    syn_can_depth = []

    xsteps = 0.007
    xmin = -0.05
    xmax = 0.15
    ysteps = 0.007
    ymin = -0.17
    ymax = 0.04

    syn_can_x = np.arange(xmin,xmax,xsteps)
    syn_can_y = np.arange(ymin,ymax,ysteps)

    syn_can_matrix=[syn_can_x[0],syn_can_y[0],0.42]
    for i in range(0,len(syn_can_x)):
        for n in range(0,len(syn_can_y)):
            row = [syn_can_x[i],syn_can_y[n],0.42]
            syn_can_matrix = np.vstack([syn_can_matrix, row])

    return syn_can_matrix

def can_grid_division(data, n_div):
    print("Dividing in grids...")
    x_thrs = []
    y_thrs = []
    grids = []
    x = data[:,0]
    y = data[:,1]

    min_x = min(data[:,0])
    max_x = max(data[:,0])
    min_y = min(data[:,1])
    max_y = max(data[:,1])
    min_z = min(data[:,2])
    max_z = max(data[:,2])
    x_thr = (max_x - min_x)/n_div
    y_thr = (max_y - min_y)/n_div
    z_thr = (max_z - min_z)/n_div

    ## Grids
    ## Get XY thresholds based on given number divisions
    for n in range(n_div):
        next_x_thr = min_x + (x_thr*n)
        x_thrs.append(next_x_thr)
        next_y_thr = min_y + (y_thr*n)
        y_thrs.append(next_y_thr)
    x_thrs.append(max_x)
    y_thrs.append(max_y)
    if(print_info):
        print("X threshohld: ", x_thrs, " / Y threshohld: ", y_thrs)

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
    print("Dividing in grids...")
    grids = []

    ## Cluster different grids
    for n in range(n_div):
        grid = data[x_thrs[n]<data[:,0]]
        grid = grid[x_thrs[n+1]>grid[:,0]]
        for b in range(n_div):
            gridy = grid[y_thrs[b]<grid[:,1]]
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
    for i in range(len(grids)):
        grid_sizes.append(len(grids[i]))

    depth = data[:,2]
# Get mean depth (or min?)
    mean_depth = np.mean(depth)
    min_depth = min(depth)

    if(print_info):
        print("Canonical mean: ", mean_depth)
        print("Canonical min: ", min_depth)

    return min_depth, mean_depth, grid_sizes

def deformation_metric(can_grids, can_min_depth, obj_data, grids):
    print("Computing deformation metric...")
    means = []
    suma = 0
    length = 300
    transl_data = []
    obj_depth = obj_data[:,2]
    max_depth=max(obj_depth)
    if(max_depth<0.6):
        max_depth_trasl = max_depth-can_min_depth
    else:
        print("max depth > 1: ")
        max_depth_trasl = 0.58-can_min_depth # 0.65 should be max posible depth (half size cloth)
    print("Max depth: ", max_depth, " / ", max_depth_trasl)

    for l in range (len(grids)):
        #mean1 = np.mean(grids[l][:,2])
        #mean1 = mean1-0.42
        depth = grids[l][:,2]
        new_grid = grids[l]
        ## Depth points traslation
        suma = 0
        for i in range(len(depth)):
            #point = (depth[i]-can_min_depth)/(1-can_min_depth)
            point = depth[i] - can_min_depth #0.42
            suma += point
            new_point=[new_grid[i][0], new_grid[i][1], point]
            transl_data.append(new_point)

        print("sum visible depths: ", suma)
        ## Fill empty points
        if len(grids[l]) < len(can_grids[l]):
            length = len(can_grids[l])
        #if len(grids[l]) < length:
            #print("Filling empty points! l=",l)
            dif = length-len(grids[l])
            suma += dif*max_depth_trasl
            print("Dif length: ", dif, " (", l,")")
        else:
            #length = len(can_grids[l])
            length = len(grids[l])
        ## Grid depth mean
        #print("SUMA / Length: ", suma, " / ", length)
        dif_to_mean = suma/length
        means.append(dif_to_mean)

    print("Means: ", means)
    if(print_info):
        print("Means: ", means)

    return means, transl_data

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
can_data = create_canonical()
can_x_grid_divs, can_y_grid_divs, can_grids = can_grid_division(can_data, n_div) # Divide garment in grid
can_min_depth, can_mean_depth, grid_sizes = canonical_params(can_data, can_grids) # Get canonical parameters
can_def_measures, can_transl_data = deformation_metric(can_grids, can_min_depth, can_data, can_grids) # Compute deformation metrics (grids depth mean)
#if(show_plot):
    #plot(can_transl_data, "transl_canonical")
    #plot(can_data, "CANONICAL")
    #plot_with_info(can_transl_data, can_transl_data, can_grids, can_x_grid_divs, can_y_grid_divs, can_min_depth, can_def_measures, "hola")
if(save_grid_sizes):
    write_grid_sizes("o1_gr_syn_can", can_grids)


# ## With REAL Canonical object
# #print("Directory:. ", can_obj_file)
# can_pcd = o3d.io.read_point_cloud(can_obj_file)
# can_data = np.asarray(can_pcd.points)
# print(can_data.shape)
#
# can_x_grid_divs, can_y_grid_divs, can_grids = can_grid_division(can_data, n_div) # Divide garment in grid
# can_min_depth, can_mean_depth, grid_sizes = canonical_params(can_data, can_grids) # Get canonical parameters
# can_def_measures, can_transl_data = deformation_metric(can_grids, can_min_depth, can_data, can_grids) # Compute deformation metrics (grids depth mean)
# #plot(can_transl_data, "transl_canonical")
# #plot(can_data, "CANONICAL")


if not all_files :
    ######## For one unique experiment
    print("Getting experiment file: ", pcd_file)
    obj_pcd = o3d.io.read_point_cloud(pcd_dir) #Read pcd files from folder
    obj_data = np.asarray(obj_pcd.points)
    plot(obj_data, "EXPERIMENT")
    ## Divide grids
    obj_grids = grid_division(obj_data, can_x_grid_divs, can_y_grid_divs, n_div) #Divide grids
    ## Compute deformation metrics (grids depth mean)
    def_measures, obj_transl_data = deformation_metric(can_grids, can_min_depth, obj_data, obj_grids) # Compute deformation metrics (grids depth mean)
    plot(obj_transl_data, "transl exp")
    ## Save results
    if(save_def_metric): ##Save means in csv
        ##Write CSV headers
        headers = ["File"]
        for i in range(0, n_div*n_div):
            headers.append(metrics_name[i])
        #wr.writerow(headers)
        #save_mean_values(pcd_file.replace("_seg.pcd", ""), n_exp, def_measures)
    if(save_grid_sizes): ##Save grid sizes
        write_grid_sizes(pcd_file.replace("_seg.pcd", ""), obj_grids)
    if(show_plot):
        plotname = save_plots_dir + pcd_file.replace("_seg.pcd", "_plot.png")
        plot_with_info(can_transl_data, obj_transl_data, obj_grids, can_x_grid_divs, can_y_grid_divs, can_min_depth, def_measures, plotname)


    ######### For all experiments
if(all_files):
    ##Read pcd files from folder
    print("Reading PCD files")
    print(data_directory)
    ##Write CSV headers
    if(save_def_metric):
        headers = ["File","Class_GT"]
        for i in range(0, n_div*n_div):
            headers.append(metrics_name[i])
        wr.writerow(headers)
    for filename in sorted(os.listdir(data_directory)):
        f = os.path.join(data_directory, filename)
        if os.path.isfile(f) and filename.endswith('.pcd'):
            print(f)
            obj_pcd = o3d.io.read_point_cloud(f)
            obj_data = np.asarray(obj_pcd.points)
            #print(len(obj_data))
            ##Divide grids
            obj_grids = grid_division(obj_data, can_x_grid_divs, can_y_grid_divs, n_div) #Divide grids
            ## Compute deformation metrics (grids depth mean)
            def_measures, obj_transl_data = deformation_metric(can_grids, can_min_depth, obj_data, obj_grids)
            if(save_def_metric): ##Save means in csv
                save_mean_values(filename.replace("_seg.pcd", ""), n_exp, def_measures)
                n_exp+=1
            if(save_grid_sizes): ##Save grid sizes
                write_grid_sizes(filename.replace("_seg.pcd", ""), obj_grids)
            if(show_plot):
                plotname = save_plots_dir + filename.replace("_seg.pcd", ".jpg")
                plot_with_info(can_transl_data, obj_transl_data, obj_grids, can_x_grid_divs, can_y_grid_divs, can_min_depth, def_measures, plotname)

## PLOTS
## Plot full garment
#plot(can_data, "./plots/canonical.html")
#plot(obj_data, "./plots/garment.html")

## Plot grids
#file_n = str(n)+str(b)+".html"
#plot(grid2,file_n)



##### TO DO
# OK- Read PCD files one at a time
# OK- Write def metric in csv file
# OK- Classification

##### REFS
# https://stackoverflow.com/questions/69372448/plotly-vertical-3d-surface-plot-in-z-x-plane-not-showing-up
# https://stackoverflow.com/questions/62403763/how-to-add-planes-in-a-3d-scatter-plot
