import numpy as np
import open3d as o3d
import plotly
import plotly.graph_objs as go


## Parameters
n_div = 2 # Division
print_info=True

## Canonical object
can_obj_file = "o1_gr_can.pcd"
obj_file = "o1_gr_can.pcd"
#obj_file = "o1_gr_e1.pcd"


def plot(data, file_name):
    fig1 = go.Scatter3d(x=data[:,0], y=data[:,1], z=data[:,2], marker=dict(opacity=1, reversescale=True, colorscale='Blues', size=5), mode='markers')
    mylayout = go.Layout(scene=dict(xaxis=dict(title="X"), yaxis=dict(title="Y"), zaxis=dict(title="Depth")))
    plotly.offline.plot({"data": [fig1], "layout": mylayout}, filename=file_name, auto_open=True)

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
        print("Grid1 size: ", len(grids[3]))
        print("Grid2 size: ", len(grids[1]))
        print("Grid3 size: ", len(grids[2]))
        print("Grid4 size: ", len(grids[0]))

    return grids

def canonical_params(data, grids):
    grid_sizes = []
    for i in range(len(grids)):
        grid_sizes.append(len(grids[i]))

    depth = data[:,2]
    mean_depth = np.mean(depth)
    min_depth = min(depth)

    if(print_info):
        print("Canonical mean: ", mean_depth)
        print("Canonical min: ", min_depth)

    return mean_depth, grid_sizes

def deformation_metric(can_grids, can_mean_depth, grids):
    means = []
    suma = 0

    for l in range (len(grids)):
        depth = can_grids[l][:,2]
        ## Depth points traslation
        for i in range(len(depth)):
            #point = (depth[i]-can_min_depth)/(1-can_min_depth)
            point = depth[i] - can_mean_depth
            suma += point
        ## Fill empty points
        if len(grids[l]) < len(can_grids[l]):
            print("Filling empty points!")
            dif = len(can_grids[l])-len(grids[l])
            suma += dif
        ## Grid depth mean
        mean = suma/len(can_grids[l])
        means.append(mean)
    
    if(print_info):
        print("Means: ", means)

    return means

##############################

## Canonical object
can_pcd = o3d.io.read_point_cloud(can_obj_file)
can_data = np.asarray(can_pcd.points)

## Divide garment in grid
can_grids = grid_division(can_data, n_div)

## Get canonical parameters
can_mean_depth, grid_sizes = canonical_params(can_data, can_grids)


## For each experiment
##Read pcd files from folder
obj_pcd = o3d.io.read_point_cloud(obj_file)
obj_data = np.asarray(obj_pcd.points)
##Divide grids
obj_grids = grid_division(obj_data, n_div)

## Compute deformation metrics (grids depth mean)
means = deformation_metric(can_grids, can_mean_depth, obj_grids)
##Save means in csv

## Get mean depth (or min?)


## PLOTS
## Plot full garment
plot(can_data, "canonical.html")
plot(obj_data, "garment.html")

## Plot grids
#file_n = str(n)+str(b)+".html"
#plot(grid2,file_n)



##### TO DO
# Read PCD files one at a time
# Write def metric in csv file
