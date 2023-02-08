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
n_div = 2 # Division
print_info=True

## Canonical object
can_obj_file = 'o1_gr_can_seg.pcd'
filename = "o1_gr_can_seg.pcd"


## CSV file to save def metric
data_filename = "./o2_pl_2x2.csv"
my_file = open(data_filename, "wb")
wr = csv.writer(my_file, delimiter=",")



def plot(data, file_name):
    x=data[:,0]
    y=data[:,1]
    z=data[:,2]
    fig = px.scatter_3d(x=data[:,0], y=data[:,1], z=data[:,2])
    fig.show()
#    bright_blue = [[0, '#7DF9FF'], [1, '#7DF9FF']]
#    bright_pink = [[0, '#FF007F'], [1, '#FF007F']]
#    light_yellow = [[0, '#FFDB58'], [1, '#FFDB58']]
#    
#    # need to add starting point of 0 to each dimension so the plane extends all the way out
##    zero_pt = pd.Series([0])
##    z = zero_pt.append(data[:,0], ignore_index = True).reset_index(drop = True)
##    y = zero_pt.append(data[:,1], ignore_index = True).reset_index(drop = True)
##    x = zero_pt.append(data[:,2], ignore_index = True).reset_index(drop = True)
#    
#    length_data_y = len(data[:,1])
#    length_data_z = len(data[:,2])
#    x_plane_pos = 1*np.ones((length_data_y,length_data_z))
#    
##    fig.add_trace(go.Surface(x=x, y=y_plane_pos, z=z))
##    fig.add_trace(go.Surface(x=x, y=y_plane_pos, z=z))
##    fig.add_trace(go.Surface(x=x_plane_pos, y=data[:,1], z=data[:,2], colorscale=light_yellow,  showscale=False))
##    fig.add_trace(go.Surface(x=x.apply(lambda x: 10), y=y, z = np.array([z]*length_data), colorscale= bright_blue, showscale=False))
##    fig.add_trace(go.Surface(x=x, y= y.apply(lambda x: 30), z =  np.array([z]*length_data).transpose(), colorscale=bright_pink, showscale=False)
#
##    fig.show()
#    x=data[:,0]
#    y=data[:,1]
#    z=data[:,2]
#    x = np.zeros(100)
#    y = np.linspace(-5,5,100)
#    z = np.linspace(-2,2,50)
#    plane = go.Surface(x=x, y=y, z=np.array([z]*len(x)))
#    fig = go.Figure()
#    fig.add_traces([plane])
#    fig.show()

    bright_blue = [[0, '#7DF9FF'], [1, '#7DF9FF']]
    bright_pink = [[0, '#FF007F'], [1, '#FF007F']]
    y=np.linspace(min(x),max(x),100)
    x=np.zeros(len(y))
    z=np.linspace(min(z),max(z),50)
    plane = go.Surface(x=y, y=x, z=np.array([z]*len(x)).T, colorscale=bright_pink, opacity=0.6)
##    fig = go.Figure()
    fig.add_traces(plane)
#    fig = go.Figure(data=[go.Surface(x=y, y=x, z=np.array([z]*len(x)).T)])
##    fig.add_trace(go.Surface(x=x, y=y, z=z))
    fig.show()
###    fig1 = px.scatter_3d(x=data[:,0], y=data[:,1], z=data[:,2])#, marker=dict(opacity=1, reversescale=True, colorscale='Blues', size=5), mode='markers')
##    mylayout = go.Layout(scene=dict(xaxis=dict(title="X"), yaxis=dict(title="Y"), zaxis=dict(title="Z")))
##    plotly.offline.plot({"data": [fig], "layout": mylayout}, filename=file_name, auto_open=True)


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

    return grids

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
        mean = suma/length
        means.append(mean)
    
    if(print_info):
        print("Means: ", means)

    return means

##############################

## Canonical object
print("Directory:. ", can_obj_file)
can_pcd = o3d.io.read_point_cloud(can_obj_file)
can_data = np.asarray(can_pcd.points)

## Divide garment in grid
can_grids = grid_division(can_data, n_div)

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
##        plot(obj_data, plotname)


## For one unique experiment
##Read pcd files from folder
obj_pcd = o3d.io.read_point_cloud(filename)
obj_data = np.asarray(obj_pcd.points)
##Divide grids
obj_grids = grid_division(obj_data, n_div)
## Compute deformation metrics (grids depth mean)
means = deformation_metric(can_grids, can_mean_depth, obj_grids)
##Save means in csv
save_mean_values(filename, means)
plotname = "./plots/" + filename + ".html"
plot(obj_data, plotname)


## PLOTS
## Plot full garment
#plot(can_data, "./plots/canonical.html")
#plot(obj_data, "./plots/garment.html")

## Plot grids
#file_n = str(n)+str(b)+".html"
#plot(grid2,file_n)



##### TO DO
# Read PCD files one at a time
# Write def metric in csv file
