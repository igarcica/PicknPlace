## This code measures grid metric of placed data

## 1. Subscribes to /segment_table/place topic to receive segmented placed cloth
## 2. Translates data to have the points near the table to 0 and the rest positive depth values
## 3. Computes grid metric
## 4. Substracts the object's thickness to the resulting grid metric tso it is agnostic to the object dimensions (otherwise towel will have larger values with less deformation)
## In process- Measure placing quality (global mean/median of depth, grid metric)
## In process- Compute grid mean matrix distance to GT matrix (0 deformation) - Used to draw a plot of deformation for each object, fold case and grasp

## How to compute alignment of pile??
## What value put when there are no points?

import numpy as np
# import os
# import csv
# import open3d as o3d
import statistics as sts
import plotly.express as px
import plotly.graph_objs as go
# from scipy.spatial.distance import cdist
# import math
import rospy


# colorscale = px.colors.sample_colorscale("jet_r", np.linspace(0, 0.3, 256))
# colorscale = px.colors.sample_colorscale("jet", np.linspace(0.6, 1, 256)) # placed object
colorscale = px.colors.sample_colorscale("jet_r", np.linspace(0, 0.4, 256)) # pile

##################################################################################################
## UTIL FUNCTIONS

def print_info(activate, arg1, arg2="", arg3="", arg4="", arg5="", arg6="", arg7=""):
    if(activate):
        print(str(arg1) + str(arg2) + str(arg3) + str(arg4) + str(arg5) + str(arg6) + str(arg7))

## Plots data inside the given plot scale
def plot(data, file_name, scale, scale_color):
    data = np.array(data)
    fig = px.scatter_3d(x=data[:,0], y=data[:,1], z=data[:,2], color=data[:,2])
    ##plotly.offline.plot({"data": [fig1], "layout": mylayout}, auto_open=True)
    ##fig.update_layout(scene=dict(zaxis=dict(range=[0.8, 0.5]), xaxis=dict(range=[0.3, 0]), yaxis=dict(range=[0.2, -0.2]) ))
    # fig.update_coloraxes(cmax=0.0, cmin=-0.2)
    fig.update_layout(scene=scale)
    fig.update_coloraxes(cmin=scale_color[0], cmax=scale_color[1])

    if not all_files:
        fig.show()
    if save_csv:
        filename = write_dir + file_name + ".jpg"
        fig.write_image(filename)

## Saves RGB images with the corresponding filename, GT class and metrics
def plot_with_info(data, x_grid_divs, y_grid_divs, can_edges, obj_thickn, n_objs, pile_thickn, scale, scale_color):
    print("\033[94m Plotting with info... \033[0m")
    data = np.array(data)
    planes_x = []
    planes_y = []
    canonic_plane = []
    x_data=data[:,0]
    y_data=data[:,1]
    z_data=data[:,2]
    bright_blue = [[0, '#7DF9FF'], [1, '#7DF9FF']]
    bright_pink = [[0, '#FF007F'], [1, '#FF007F']]
    bright_orange = [[0, '#FFA500'], [1, '#FFA500']]

    # Plot garment
    fig = px.scatter_3d(x=data[:,0], y=data[:,1], z=data[:,2], color=data[:,2])
    fig.update_traces(marker=dict(color="teal"))

    # Plot X axis divisions
    for n in range(1,len(x_grid_divs)-1):
        x=x_grid_divs[n]*np.ones(len(x_data))
        y=np.linspace(min(y_data),max(y_data),100)
        z=np.linspace(min(z_data)-0.01,max(z_data)+0.01,50)
        plane = go.Surface(x=x, y=y, z=np.array([z]*len(x)), colorscale=bright_orange, opacity=0.5)
        planes_x.append(plane)
    # Plot Y axis divisions
    for n in range(1,len(y_grid_divs)-1):
        x=np.linspace(min(x_data),max(x_data),100)
        y=y_grid_divs[n]*np.ones(len(y_data))
        z=np.linspace(min(z_data)-0.01,max(z_data)+0.01,50)
        plane = go.Surface(x=x, y=y, z=np.array([z]*len(x)).T, colorscale=bright_orange, opacity=0.5)
        planes_y.append(plane)

    # Plot Canonical plane
    x=np.linspace(can_edges[0],can_edges[1],100)
    y=np.linspace(can_edges[2],can_edges[3],50)
    # z=(obj_thickn*n_objs)*np.ones(len(y_data))
    z=pile_thickn*np.ones(len(y_data))
    # z=0.05*np.ones(len(y_data))
    canonic = go.Surface(x=x, y=y, z=np.array([z]*len(x)).T, colorscale=bright_pink, opacity=0.3)
    canonic_plane.append(canonic)

    # fig.add_traces(data)
    fig.add_traces(planes_x)
    fig.add_traces(planes_y)
    fig.add_traces(canonic_plane)
    fig.update_layout(scene=dict(zaxis=dict(range=[0, 0.2]), xaxis=dict(range=[0, 0.4]), yaxis=dict(range=[0.2, -0.25]), aspectratio=dict(x=1, y=1, z=1) ))
    # fig.update_coloraxes(cmax=0.12, cmin=0.0)
    # fig.update_layout(scene=scale)
    fig.update_coloraxes(cmin=scale_color[0], cmax=scale_color[1])
    fig.update_layout( # Increase font sizes
    scene=dict(
        xaxis=dict(title="X Axis", titlefont=dict(size=35), tickfont=dict(size=16)),  # Increase X labels
        yaxis=dict(title="Y Axis", titlefont=dict(size=35), tickfont=dict(size=16)),  # Increase Y labels
        zaxis=dict(title="Z Axis", titlefont=dict(size=35), tickfont=dict(size=16))   # Increase Z labels
    ),
    # font=dict(size=16)  # Increase general font size
    )    

    # fig = go.Figure(data=[go.Scatter3d(
    # x=data[:,0], 
    # y=data[:,1],
    # z=data[:,2],
    # mode='markers',
    # marker=dict(
    #     # size=12,
    #     color=data[:,2],                # set color to an array/list of desired values
    #     colorscale=colorscale,   # choose a colorscale
    #     # opacity=0.8
    # )
    # )])
    # fig.update_layout(scene=dict(zaxis=dict(range=[0, 0.2]), xaxis=dict(range=[0, 0.4]), yaxis=dict(range=[0.2, -0.25]), aspectratio=dict(x=1, y=1, z=1) ))
    # fig.update_layout( # Increase font sizes
    # scene=dict(
    #     xaxis=dict(title="X Axis", titlefont=dict(size=35), tickfont=dict(size=16)),  # Increase X labels
    #     yaxis=dict(title="Y Axis", titlefont=dict(size=35), tickfont=dict(size=16)),  # Increase Y labels
    #     zaxis=dict(title="Z Axis", titlefont=dict(size=35), tickfont=dict(size=16))   # Increase Z labels
    # ))
    

    return fig

## Plot metrics in colored grid
# def plot_metrics(metrics, scale_color):
def plot_metrics(metrics, obj_thickn, n_objs):
    metrics = np.array(metrics)
    # metrics = [[0.08, 0.08, 0.08], [0.08, 0.08, 0.08], [0.08, 0.08, 0.08]]
    # metrics = [[0.08, 0.08, 0.1], [0.096, 0.092, 0.08], [0.09, 0.1, 0.094]]
    div=int(np.sqrt(len(metrics)))
    metrics = metrics.reshape(div,div)
    # fig = px.imshow(metrics, text_auto=True, labels=dict(x='x', y='y'))
    fig = px.imshow(metrics, text_auto=True, color_continuous_scale=colorscale) #text_auto=True, labels=dict(x='x', y='y'))
    # fig.update_coloraxes(cmin=scale_color[0], cmax=scale_color[1])#cmax=0.08, cmin=0.0)
    # fig.update_coloraxes(cmin=0.0, cmax=0.1) #0.2

    min_depth = obj_thickn * n_objs
    max_depth = (obj_thickn*3) + (min_depth-obj_thickn)
    fig.update_coloraxes(cmin=min_depth, cmax=max_depth) 
    fig.update_layout(xaxis=dict(showticklabels=False), yaxis=dict(showticklabels=False))

    return fig

## Plot raw point cloud
def plot_raw_data(data):
    x_min = min(data[:,0])
    x_max = max(data[:,0])
    y_min = min(data[:,1])
    y_max = max(data[:,1])
    z_min = min(data[:,2])
    z_max = max(data[:,2])
    
    scale = dict(zaxis=dict(range=[z_min, z_max]), xaxis=dict(range=[x_min, x_max]), yaxis=dict(range=[y_min, y_max]), aspectratio=dict(x=1, y=1, z=1) )
    scale_color = [z_min, z_max]
    # fig = plot(data, "raw", scale, scale_color)
    data = np.array(data)
    fig = px.scatter_3d(x=data[:,0], y=data[:,1], z=data[:,2], color=data[:,2])
    fig.update_layout(scene=scale)
    fig.update_coloraxes(cmin=scale_color[0], cmax=scale_color[1])

    # print_info(activate_print, "xmin: ", x_min, "xmax: ", x_max)
    # print_info(activate_print, "ymin: ", y_min, "ymax: ", y_max)
    # print_info(activate_print, "zmin: ", z_min, "zmax: ", z_max)

    return fig

def save_data_values(exp_name, data_values, data_values2):
    print("\033[94m Writing deformation metric values... \033[0m")
    data = []
    data.append(exp_name)
    for i in range(len(data_values)):
        data.append(data_values[i])
    # data.append(data_values)
    means_data_wr.writerow(data)

    data2 = []
    data2.append(exp_name)
    for i in range(len(data_values2)):
        data2.append(data_values2[i])
    dist_data_wr.writerow(data2)

def save_data(csv_file_wr, exp_name, data_values):
    print("\033[94m Writing csv data... \033[0m")
    data = []
    data.append(exp_name)
    for i in range(len(data_values)):
        data.append(data_values[i])
    csv_file_wr.writerow(data)


##################################################################################################
## DATA PROCESS FUNCTIONS

##Removes noise points based on a box threshold (removes table points)
def filter_sample(data, filter_box):
    
    x_thrs = np.array(filter_box[0])
    filtered_sample = data[data[:,0]<x_thrs[1]] ##Filter table points (depth axis)

    return filtered_sample

## Moves pointcloud to have points near the table 0
def translate_data(data, cam_to_table):
    ## Traslate depth (0(gripper)-deformation)
    depth = data[:,0]
    transl_data = []
    not_pile_data = []
    suma = 0
    for i in range(len(depth)):
        #point = (depth[i]-can_min_depth)/(1-can_min_depth)
        point = cam_to_table - depth[i] # new points should positive from 0 to deformation
        suma += point
        new_point=[data[i,2], data[i,1], point] #changed axis to have z as depth
        transl_data.append(new_point)

    transl_data = np.array(transl_data)
    transl_depth = transl_data[:,2]
    # transl_depth = np.array(transl_data)[:,2]
    mean = sts.mean(transl_depth)
    median = sts.median(transl_depth)
    metrics = [mean, median]

    return transl_data, metrics

## Normalizes pointcloud, where 0 is the table position + the object/pile thickness and 1 is largest size (?) - what can we assign as the max deformation?
def normalize_transl_data(data):
    obj_edge_size = CLOTH_SIZE.get(obj_name, None) #Get object dimensions
    min_depth = obj_edge_size[2] #Object's thickness should be 0 deformation
    max_depth = obj_edge_size[0]/2 #half of the long edge (is unlikely to be placed vertically)
    depth = data[:,2]
    norm_transl_data = []
    for i in range(len(depth)):
        # point = depth[i]/max_depth
        point = (depth[i]-min_depth)/max_depth
        new_point=[data[i,0], data[i,1], point] #Maintain axis from transl_data
        norm_transl_data.append(new_point)

    norm_transl_data = np.array(norm_transl_data)
    norm_transl_depth = norm_transl_data[:,2]
    mean = sts.mean(norm_transl_depth)
    median = sts.median(norm_transl_depth)
    print("Global Mean: ", mean)
    print("Global Median: ", median)
    norm_metrics = [mean, median]

    return norm_transl_data, norm_metrics

## Obtain canoncial parameters to compute grid threshold
def create_canonical(n_div, gripper_position, grasped_edge_size, nongrasped_edge_size):
    # print("\033[96m Creating canonical \033[0m")
    rospy.loginfo("Placing_quality: Creating canonical")

    xmin = xmax = ymin = ymax = 0
    x_thrs = []
    y_thrs = []
    
    # ymin = gripper_position[1]-(obj_edge_size[grasped_edge]/2)
    # ymax = gripper_position[1]+(obj_edge_size[grasped_edge]/2)
    # xmax = gripper_position[0] 
    # xmin = gripper_position[0]-obj_edge_size[non_grasped_edge]
    ymin = gripper_position[1]-(grasped_edge_size/2)
    ymax = gripper_position[1]+(grasped_edge_size/2)
    xmax = gripper_position[0] 
    xmin = gripper_position[0]-nongrasped_edge_size

    x_thr = (xmax - xmin)/n_div
    y_thr = (ymax - ymin)/n_div
    
    canonical_edges = [xmin, xmax, ymin, ymax]

    ## Grids
    ## Get XY thresholds based on given number divisions
    x_thrs.append(xmin-1)
    y_thrs.append(ymin-1)
    for n in range(1,n_div):
        next_x_thr = xmin + (x_thr*n)
        x_thrs.append(next_x_thr)
        next_y_thr = ymin + (y_thr*n)
        y_thrs.append(next_y_thr)
    x_thrs.append(xmax+1)
    y_thrs.append(ymax+1)

    return x_thrs, y_thrs, canonical_edges

## Obtains grid point clouds of data
def grid_division(data, x_thrs, y_thrs, n_div):
    # print("\033[96m Dividing in grids... \033[0m")
    rospy.loginfo("Placing_quality: Dividing in grids")
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

    return grids

## TO DELETE- Computes grid divisions
def divide_points_into_grid(data, x_min, x_max, y_min, y_max):
    # Define the boundaries for the 3x3x3 grid
    x_edges = np.linspace(x_min, x_max, 4)
    y_edges = np.linspace(y_min, y_max, 4)
    print("linspace: ", x_edges, " / ", y_edges)

    # Initialize the clusters
    clusters = [[] for _ in range(9)]

    # Assign each point to the appropriate cluster
    for point in data:
        x, y, z = point

        # Determine the x cluster index
        if x < x_edges[1]:
            x_idx = 0
        elif x < x_edges[2]:
            x_idx = 1
        else:
            x_idx = 2

        # Determine the y cluster index
        if y < y_edges[1]:
            y_idx = 0
        elif y < y_edges[2]:
            y_idx = 1
        else:
            y_idx = 2
        
        # Calculate the cluster index
        cluster_idx = x_idx * 3 + y_idx

        # Add the point to the appropriate cluster
        clusters[cluster_idx].append(point)

    # Convert the clusters to numpy arrays
    clusters = [np.array(cluster) for cluster in clusters]

    print(len(clusters[4]))

    plot(clusters[4], "hola", plot_scale, plot_scale_color)

    return clusters

## Computes mean of each grid section
def def_metric(grids, grasp_edge_size, obj_thickness):

    # obj_thickness = obj_dims[2] #obtained from CLOTH_SIZE
    max_depth = grasp_edge_size/2
    means = []

    ## For each section of the grid
    for l in range (len(grids)):
        length = len(grids[l])
        ## If there are no points in the grid, then the mean is max deformation
        if(length == 0):
            means.append(max_depth) #To check - should be the same as the max of the bad_placement in placing_qual()
        ## If the grid is not empty, compute mean of depth
        else:
            depth = grids[l][:,2]
            new_grid = grids[l]
            grid_mean = sts.mean(depth)
            means.append(grid_mean)
    
            # #Instead of normalizing the data with the thickness (what will bias the data), we substract the thickness to the resulting metric
            # if piling:
            #     grid_def = grid_mean-(obj_thickness*2)
            # else:
            #     grid_def = grid_mean-obj_thickness
            # norm_means.append(grid_def) 
            # #What if grid_def is negative?

    return means

def placing_qual(metrics, n_div, grasp_edge_size, obj_thickness, n_objs):

    min_depth = obj_thickness * n_objs #Object/pile thickness should be 0 deformation
    # max_depth = grasp_edge_size/2 + (min_depth-obj_thickness) + 0.01 #Max depth occurs when the cloth is folded by half (grasped edge size /2) + the piled object thickness (0 for placed, obj thick for pile)
    # half_max_depth = max_depth/2
    half_max_depth = min_depth+0.01
    max_depth = (obj_thickness*3) + (min_depth-obj_thickness)
    print("min depth: ", min_depth, " / max depth: ", max_depth)

    metrics = np.array(metrics)
    flat_placement = min_depth*np.ones(n_div*n_div)
    # bad_placement = np.array([[0.05, 0.1, 0.05], [0.05, 0.1, 0.05], [0.05, 0.1, 0.05]]) #to check which is the most representative
    # bad_placement = np.array([[half_max_depth, max_depth, half_max_depth], [half_max_depth, max_depth, half_max_depth], [half_max_depth, max_depth, half_max_depth]]) #to check which is the most representative ##for pillowcase
    # bad_placement = np.array([[max_depth, max_depth, max_depth], [max_depth, max_depth, max_depth], [max_depth, max_depth, max_depth]])
    # bad_placement = 0.16*np.ones(n_div*n_div) #for towel (system adaptability)
    bad_placement = max_depth*np.ones(n_div*n_div) #for piles of towels
    bad_placement = bad_placement.reshape(-1, 1)
    print("FLAT MATRIX: ", flat_placement)

    max_dist = np.linalg.norm(bad_placement - flat_placement, 1) #Max distance from bad placement to perfect placement (100% error) - Used for normalization
    print("BAD MATRIX: ", bad_placement)
    print("Max distance: ", max_dist)
    dist = np.linalg.norm(metrics - flat_placement, 1) #Ditance of current sample to perfect placement
    print("Distance", dist)
    # placing_error = (dist/max_dist)*100 # Normalize distance
    placing_error = (dist-min_depth)/(max_dist-min_depth)*100 # Normalize distance
    placing_quality = 100-placing_error # Get placing quality (not error)
    rospy.loginfo("Placing_quality: Placing quality %f ", placing_quality)

    return placing_quality
    
#test - placing quality for piles of different objects (different objects thickness)
def dif_objs_placing_qual(metrics, n_div, grasp_edge_size, pile_thickness, n_objs):

    obj_thickness = pile_thickness/n_objs
    min_depth = pile_thickness  #Pile thickness should be 0 deformation
    # max_depth = (obj_thickness*3) + (min_depth-obj_thickness)
    max_depth = min_depth + (obj_thickness*3) #The pile height + two more objectss
    print("min depth: ", min_depth, " / max depth: ", max_depth)

    metrics = np.array(metrics)
    flat_placement = min_depth*np.ones(n_div*n_div)
    bad_placement = max_depth*np.ones(n_div*n_div) #for piles of towels
    bad_placement = bad_placement.reshape(-1, 1)
    print("FLAT MATRIX: ", flat_placement)

    max_dist = np.linalg.norm(bad_placement - flat_placement, 1) #Max distance from bad placement to perfect placement (100% error) - Used for normalization
    print("BAD MATRIX: ", bad_placement)
    print("Max distance: ", max_dist)
    dist = np.linalg.norm(metrics - flat_placement, 1) #Ditance of current sample to perfect placement
    print("Distance", dist)
    placing_error = (dist-min_depth)/(max_dist-min_depth)*100 # Normalize distance
    placing_quality = 100-placing_error # Get placing quality (not error)
    rospy.loginfo("Placing_quality: Placing quality %f ", placing_quality)

    return placing_quality
    

##################################################################################################
### INFO
## grasping_grid_metric_ros and placing_grid_metric_ros could be joined.
## Grasping pointcloud data for deformation class and Placing pointcloud data for placing quality are processed and used differently
## but some functions are similar (filter_sample, plots, def metric).