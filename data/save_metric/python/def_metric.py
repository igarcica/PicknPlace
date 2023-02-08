import os
import csv
import numpy as np
import pandas as pd
import plotly
import plotly.express as px
import plotly.graph_objs as go
import open3d as o3d

import matplotlib.pyplot as plt


## Parameters
n_div = 3 # Division
print_info=True

## Canonical object
can_obj_file = 'o1_gr_can_seg.pcd'
filename = "o1_gr_e10_seg.pcd"


## CSV file to save def metric
data_filename = "./o2_pl_2x2.csv"
my_file = open(data_filename, "wb")
wr = csv.writer(my_file, delimiter=",")



def plot(can, data, grids, x_grid_divs, y_grid_divs, can_mean_depth, def_measure, file_name):
    planes_x = []
    planes_y = []
    x_data=data[:,0]
    y_data=data[:,1]
    z_data=data[:,2]
    # Plot garment
    fig = px.scatter_3d(x=data[:,0], y=data[:,1], z=data[:,2], color=data[:,2])
   
    # Plot grid divisions
    bright_blue = [[0, '#7DF9FF'], [1, '#7DF9FF']]
    bright_pink = [[0, '#FF007F'], [1, '#FF007F']]

    # Plot X axis divisions
    for n in range(1,len(y_grid_divs)-1): 
        x=x_grid_divs[n]*np.ones(len(x_data))
        y=np.linspace(min(y_data),max(y_data),100)
        z=np.linspace(min(z_data),max(z_data),50)
        plane = go.Surface(x=x, y=y, z=np.array([z]*len(x)), colorscale=bright_blue, opacity=0.6)
        planes_x.append(plane)
    # Plot Y axis divisions
    for n in range(1,len(y_grid_divs)-1): 
        x=np.linspace(min(x_data),max(x_data),100)
        y=y_grid_divs[n]*np.ones(len(y_data))
        z=np.linspace(min(z_data),max(z_data),50)
        plane = go.Surface(x=x, y=y, z=np.array([z]*len(x)).T, colorscale=bright_blue, opacity=0.6)
        planes_y.append(plane)

    # Plot grids mean values
    grids = np.array(grids)
    values = []
    for i in range(len(grids)):
        cent_x = np.sum(grids[i][:,0])/len(grids[i])
        cent_y = np.sum(grids[i][:,1])/len(grids[i])
        value = [cent_x, cent_y, can_mean_depth+def_measure[i]]
        values.append(value)
    print(values)
    values = np.array(values)
    point = go.Scatter3d(x=values[:,0], y=values[:,1], z=values[:,2], marker=dict(colorscale=bright_blue))
#    value = [-0.02, -0.12, can_mean_depth+def_measure[0]]
#    values.append(value)
#    value = [-0.02, -0.03, can_mean_depth+def_measure[1]]
#    values.append(value)
#    value = [0.08, -0.12, can_mean_depth+def_measure[2]]
#    values.append(value)
#    value = [0.08, -0.03, can_mean_depth+def_measure[3]]
#    values.append(value)
    
    
    can = go.Scatter3d(x=can[:,0], y=can[:,1], z=can[:,2])
    fig.add_traces(can)
    fig.add_traces(planes_x)
    fig.add_traces(planes_y)
    fig.add_traces(point)
#    fig.update_layout(scene=dict(xaxis=dict(range=[0.2, -0.1]), yaxis=dict(range=[-0.2, 0.05]), zaxis=dict(range=[0.5, 0.4])))
    fig.show()


def save_mean_values(exp_name, mean_data):
    print("Writing mean values...")
    data = []
    #print(mean_data)
    data.append(exp_name)
    for i in range(len(mean_data)):
        data.append(mean_data[i])
    print(data)
    wr.writerow(data)

def grid_division(data, n_div):
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
    for n in range(n_div):
        next_x_thr = min_x + (x_thr*n)
        x_thrs.append(next_x_thr)
        next_y_thr = min_y + (y_thr*n)
        y_thrs.append(next_y_thr)
    x_thrs.append(max_x)
    y_thrs.append(max_y)
    if(print_info):
        print("X threshohld: ", x_thrs, " / Y threshohld: ", y_thrs)

    for n in range(n_div):
        grid = data[x_thrs[n]<data[:,0]]
        grid = grid[x_thrs[n+1]>grid[:,0]]
        for b in range(n_div):
            gridy = grid[y_thrs[b]<grid[:,1]]
            grid2 = gridy[y_thrs[b+1]>gridy[:,1]]
            #file_n = str(n)+str(b)+".html"
            #plot(grid2,file_n)
            grids.append(grid2)
    
    if(print_info):
       # print("Grid1 size: ", len(grids[3]))
       # print("Grid2 size: ", len(grids[1]))
       # print("Grid3 size: ", len(grids[2]))
       # print("Grid4 size: ", len(grids[0]))
        print("Grid1 size: ", len(grids[0]))
        print("Grid2 size: ", len(grids[1]))
        print("Grid3 size: ", len(grids[2]))
        print("Grid4 size: ", len(grids[3]))

    return x_thrs, y_thrs, grids

def canonical_params(data, grids):
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

    return mean_depth, grid_sizes

def deformation_metric(can_grids, can_mean_depth, grids):
    means = []
    suma = 0
    length = 300

    for l in range (len(grids)):
        depth = can_grids[l][:,2]
        ## Depth points traslation
        for i in range(len(depth)):
            #point = (depth[i]-can_min_depth)/(1-can_min_depth)
            point = depth[i] - can_mean_depth
            suma += point
        ## Fill empty points
#        if len(grids[l]) < len(can_grids[l]):
        if len(grids[l]) < length:
            print("Filling empty points!")
            dif = len(can_grids[l])-len(grids[l])
            suma += dif
##            length = len(can_grids[l])
#        else:
#            length = len(grids[l])
        ## Grid depth mean
        print("SUMA / Length: ", suma, " / ", length)
        dif_to_mean = suma/length
        means.append(dif_to_mean)
    
    if(print_info):
        print("Means: ", means)

    return means

##############################

## Canonical object
print("Directory:. ", can_obj_file)
can_pcd = o3d.io.read_point_cloud(can_obj_file)
can_data = np.asarray(can_pcd.points)

## Divide garment in grid
can_x_grid_divs, can_y_grid_divs, can_grids = grid_division(can_data, n_div)
## Get canonical parameters
can_mean_depth, grid_sizes = canonical_params(can_data, can_grids)

## For each experiment
#print("Reading PCD files")
#directory = './'
#for filename in sorted(os.listdir(directory)):
#    f = os.path.join(directory, filename)
#    if os.path.isfile(f) and filename.endswith('.pcd'):
#        print(filename)
#        ##Read pcd files from folder
#        obj_pcd = o3d.io.read_point_cloud(filename)
#        obj_data = np.asarray(obj_pcd.points)
#        ##Divide grids
#        obj_grids = grid_division(obj_data, n_div)
#        
#        ## Compute deformation metrics (grids depth mean)
#        means = deformation_metric(can_grids, can_mean_depth, obj_grids)
#        ##Save means in csv
#        save_mean_values(filename, means)
##        plotname = "./plots/" + filename + ".html"
##        plot(obj_data, x_grid_divs, y_grid_divs, plotname)


## For one unique experiment
##Read pcd files from folder
obj_pcd = o3d.io.read_point_cloud(filename)
obj_data = np.asarray(obj_pcd.points)
##Divide grids
x_grid_divs, y_grid_divs, obj_grids = grid_division(obj_data, n_div)
## Compute deformation metrics (grids depth mean)
def_measures = deformation_metric(can_grids, can_mean_depth, obj_grids)
##Save means in csv
save_mean_values(filename, def_measures)
plotname = "./plots/" + filename + ".png"
plot(can_data, obj_data, obj_grids, x_grid_divs, y_grid_divs, can_mean_depth, def_measures, plotname)


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
# Classification

##### REFS
# https://stackoverflow.com/questions/69372448/plotly-vertical-3d-surface-plot-in-z-x-plane-not-showing-up
# https://stackoverflow.com/questions/62403763/how-to-add-planes-in-a-3d-scatter-plot
