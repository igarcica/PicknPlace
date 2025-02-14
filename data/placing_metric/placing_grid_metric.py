## This code measures grid metric of placed data

## 1. Read PCD file of segmented placed cloth
## 2. Translates data to have the points near the table to 0 and the rest positive depth values
## 3. Computes grid metric
## 4. Substracts the object's thickness to the resulting grid metric tso it is agnostic to the object dimensions (otherwise towel will have larger values with less deformation)
## In process- Measure placing quality (global mean/median of depth, grid metric)
## In process- Compute grid mean matrix distance to GT matrix (0 deformation) - Used to draw a plot of deformation for each object, fold case and grasp

## How to compute alignment of pile??
## What value put when there are no points?

import numpy as np
import os
import csv
import open3d as o3d
import statistics as sts
import plotly.express as px
import plotly.graph_objs as go
from scipy.spatial.distance import cdist
import math


all_files = False
# data_directory ="/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/PCD_placing/"
# pcd_file = "towel_me_r_1.pcd" #pillowc_se_v_1.pcd" #towel_me_r_1.pcd"
data_directory ="/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/test/short/piled_moved/"
pcd_file = "towel.pcd"
pcd_dir = data_directory+pcd_file
write_dir = "/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/placing_metric/"

save_csv = False
activate_print = False

n_divisions = 3
cam_to_table = 0.8 ## Used in 
# gripper_position = [0.32, -0.025] ## Used to compute grid divisions (wrt ext_camera_link changing x-axis for z-axis)
gripper_position = [0.32, -0.04]
# gripper_position = [0.27, -0.07]
piling = True
grasped_by = "short"

raw_sample_filter_box = [[0, cam_to_table], [-0.2, 0.2], [0, 0.3]] #box to filter sample removing noise points wrt camera axis
plot_scale = dict(xaxis=dict(range=[0, 0.4]), yaxis=dict(range=[0.2, -0.3]), zaxis=dict(range=[0, 0.3]), aspectratio=dict(x=1, y=1, z=1) ) #plot scale for grasped samples
# plot_scale_color = [0.0, 0.1] # plot depth color scale for grasped samples
plot_scale_color = [0.0, 0.2] # plot depth color scale for grasped samples

## In the case of the placing metric, the thickness of the objects plays a role
CLOTH_SIZE = {
    "towel": (0.23,0.25, 0.04),
    "pillowc": (0.23,0.28, 0.01),
    "towel_2l": (0.45,0.5),
    "towel_4l": (0.25,0.45),
    "towel_6l": (0.25, 0.3),
    "towel_8l": (0.23,0.25, 0.05),
    "towel_12l": (0.15,0.25),
    "pillowc_2l": (0.44,0.54),
    "pillowc_4l": (0.28,0.45),
    "pillowc_6l": (0.23,0.37),
    "pillowc_8l": (0.23,0.28),
    "pillowc_12l": (0.15,0.28),
    "pillowc_16l": (0.14,0.23),
    "cotnap_2l": (0.25,0.5),
    "cotnap_4l": (0.25,0.25),
    "cotnap_6l": (0.17,0.25),
    "cotnap_8l": (0.13,0.25),
    "cotnap_12l": (0.09,0.25),
    "cotnap_16l": (0.13,0.13),
    "linenap_2l": (0.25,0.5),
    "linenap_4l": (0.25,0.25),
    "linenap_6l": (0.17,0.25),
    "linenap_8l": (0.13,0.25),
    "linenap_12l": (0.09,0.25),
    "linenap_16l": (0.13,0.13),
    "check_4l": (0.25,0.35),
    "check_6l": (0.24,0.25),
    "check_8l": (0.18,0.25),
    "check_12l": (0.12,0.25),
    "check_16l": (0.13,0.18),
    "waffle_4l": (0.25,0.35),
    "waffle_6l": (0.24,0.25),
    "waffle_8l": (0.18,0.25),
    "waffle_12l": (0.12,0.25),
    "waffle_16l": (0.13,0.18)
    }

#default grasped and non-grasped value positions from CLOTH_SIZE
# non_grasped_edge = 0
# grasped_edge = 1 #longest edge is grasped

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
def plot_with_info(data, x_grid_divs, y_grid_divs, can_edges, filename, scale, scale_color):
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
    obj_edge_size = CLOTH_SIZE.get(obj_name, None) #Get object dimensions
    obj_thickn=obj_edge_size[2]

    # Plot garment
    fig = px.scatter_3d(x=data[:,0], y=data[:,1], z=data[:,2], color=data[:,2])

    # Plot X axis divisions
    for n in range(1,len(x_grid_divs)-1):
        x=x_grid_divs[n]*np.ones(len(x_data))
        y=np.linspace(min(y_data),max(y_data),100)
        z=np.linspace(min(z_data)-0.01,max(z_data)+0.01,50)
        plane = go.Surface(x=x, y=y, z=np.array([z]*len(x)), colorscale=bright_blue, opacity=0.4)
        planes_x.append(plane)
    # Plot Y axis divisions
    for n in range(1,len(y_grid_divs)-1):
        x=np.linspace(min(x_data),max(x_data),100)
        y=y_grid_divs[n]*np.ones(len(y_data))
        z=np.linspace(min(z_data)-0.01,max(z_data)+0.01,50)
        plane = go.Surface(x=x, y=y, z=np.array([z]*len(x)).T, colorscale=bright_blue, opacity=0.4)
        planes_y.append(plane)

    # Plot Canonical plane
    x=np.linspace(can_edges[0],can_edges[1],100)
    y=np.linspace(can_edges[2],can_edges[3],50)
    z=obj_thickn*np.ones(len(y_data))
    canonic = go.Surface(x=x, y=y, z=np.array([z]*len(x)).T, colorscale=bright_pink, opacity=0.4)
    canonic_plane.append(canonic)

    # fig.add_traces(data)
    fig.add_traces(planes_x)
    fig.add_traces(planes_y)
    fig.add_traces(canonic_plane)
    # fig.update_layout(scene=dict(zaxis=dict(range=[0, 0.2]), xaxis=dict(range=[0.4, 0.05]), yaxis=dict(range=[0.2, -0.2]), aspectratio=dict(x=1, y=1, z=1) ))
    # fig.update_coloraxes(cmax=0.12, cmin=0.0)
    fig.update_layout(scene=scale)
    fig.update_coloraxes(cmin=scale_color[0], cmax=scale_color[1])

    if not all_files:
        fig.show()
    if save_csv:
        filename = write_dir + filename + ".jpg"
        fig.write_image(filename)

## Plot metrics in colored grid
def plot_metrics(filename, metrics, scale_color):
    metrics = np.array(metrics)
    div=int(np.sqrt(len(metrics)))
    metrics = metrics.reshape(div,div)
    print(metrics)
    fig = px.imshow(metrics, text_auto=True, labels=dict(x='x', y='y'))
    fig.update_coloraxes(cmin=scale_color[0], cmax=scale_color[1])#cmax=0.08, cmin=0.0)

    if not all_files:
        fig.show()
    if save_csv:
        filename = write_dir + filename + ".jpg"
        fig.write_image(filename)

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
    plot(data, "raw", scale, scale_color)

    print_info(activate_print, "xmin: ", x_min, "xmax: ", x_max)
    print_info(activate_print, "ymin: ", y_min, "ymax: ", y_max)
    print_info(activate_print, "zmin: ", z_min, "zmax: ", z_max)

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
def translate_data(data, cam_table):
    ## Traslate depth (0(gripper)-deformation)
    depth = data[:,0]
    transl_data = []
    not_pile_data = []
    suma = 0
    for i in range(len(depth)):
        #point = (depth[i]-can_min_depth)/(1-can_min_depth)
        point = cam_table - depth[i] # new points should positive from 0 to deformation
        suma += point
        new_point=[data[i,2], data[i,1], point] #changed axis to have z as depth
        transl_data.append(new_point)

    transl_data = np.array(transl_data)
    transl_depth = transl_data[:,2]
    # transl_depth = np.array(transl_data)[:,2]
    mean = sts.mean(transl_depth)
    median = sts.median(transl_depth)
    print("Global Mean: ", mean)
    print("Global Median: ", median)
    metrics = [mean, median]

    activate_print=True
    print_info(activate_print, "Y min: ", min(data[:,1]))
    print_info(activate_print, "Y max: ", max(data[:,1]))
    print_info(activate_print, "Edge Y: ", max(data[:,1])-min(data[:,1]))
    print_info(activate_print, "X min: ", min(data[:,2]))
    print_info(activate_print, "X max: ", max(data[:,2]))
    print_info(activate_print, "Edge X: ", max(data[:,2])-min(data[:,2]))
    activate_print=False

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
def create_canonical(n_div, gripper_pos, grasp_edge_size, nongrasp_edge_size):
    # print("\033[96m Creating canonical for ", obj_name, " \033[0m")
    print("\033[96m Creating canonical \033[0m")

    xmin = xmax = ymin = ymax = 0
    x_thrs = []
    y_thrs = []

    # obj_edge_size = CLOTH_SIZE.get(obj_name, None)
    
    # ymin = gripper_position[1]-(obj_edge_size[grasped_edge]/2)
    # ymax = gripper_position[1]+(obj_edge_size[grasped_edge]/2)
    # xmax = gripper_position[0] 
    # xmin = gripper_position[0]-obj_edge_size[non_grasped_edge]
    ymin = gripper_pos[1]-(grasp_edge_size/2)
    ymax = gripper_pos[1]+(grasp_edge_size/2)
    xmax = gripper_pos[0] 
    xmin = gripper_pos[0]-nongrasp_edge_size

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

    print_info(activate_print, xmin, " / ", xmax, " / ", ymin, " / ", ymax)
    print_info(activate_print, x_thrs, " / ", y_thrs)

    return x_thrs, y_thrs, canonical_edges

## Obtains grid point clouds of data
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
def def_metric(grids, grasp_edge_size):

    # obj_thickness = obj_dims[2] #obtained from CLOTH_SIZE
    max_depth = grasp_edge_size/2
    means = []
    norm_means = []
    dev_means = []
    ## For each section of the grid
    for l in range (len(grids)):
        length = len(grids[l])
        print_info(activate_print, "\033[94m Grid length \033[0m", length)
        ## If there are no points in the grid, then the mean is max deformation
        if(length == 0):
            # means.append(-obj_dims[non_grasped_edge]+0.05/2) #Max depth (should be 1 when normalized). +5cm to give margin
            means.append(max_depth)
            norm_means.append(1) 
            dev_means.append(1)
        ## If the grid is not empty, compute mean of depth
        else:
            depth = grids[l][:,2]
            new_grid = grids[l]
            grid_mean = sts.mean(depth)
            print_info(activate_print, "Grid mean: ", grid_mean)
            means.append(grid_mean)
            dev_means.append(np.std(depth))
    
            # # #Instead of normalizing the data with the thickness (what will bias the data), we substract the thickness to the resulting metric
            # # if piling:
            # #     grid_def = grid_mean-(obj_thickness*2)
            # # else:
            # #     grid_def = grid_mean-obj_thickness
            # # norm_means.append(grid_def) 
            # # #What if grid_def is negative?

            # #Instead of normalizing the data with the thickness (what will bias the data), we substract the thickness to the resulting metric
            # min_depth = obj_dims[2] #Object's thickness should be 0 deformation
            # max_depth = obj_dims[0]/2 #half of the long edge (is unlikely to be placed vertically)
            # if piling:
            #     # grid_def = grid_mean-(obj_thickness*2)
            #     point = grid_mean/(max_depth+min_depth)
            #     # grid_def = (grid_mean-(min_depth*2))/0.1
            # else:
            #     # grid_def = grid_mean-obj_thickness
            #     # grid_def = grid_mean/max_depth
            #     grid_def = (grid_mean-min_depth)/(max_depth-min_depth)
            # norm_means.append(grid_def) 
            # #What if grid_def is negative?
            

            ###DEVIATION METRICS

    # print("Means: ", means)
    # print("Norm means: ", norm_means)
    # print("Mean means: ", sts.mean(means))
    # print("Mean norm means: ", sts.mean(norm_means))

    return means

# def distan(metrics, n_div):
#     distances = []

#     metrics = np.array(metrics)
#     metrics = metrics.reshape(-1, 1)
#     # hola = listofzeros = [0] * n_div*n_div
#     # print(hola)
#     gt_matrix = 0.03*np.ones(n_div*n_div)
#     gt_matrix = gt_matrix.reshape(-1, 1) 
#     # print(type(metrics))
#     # print(type(hola))
#     # print(gt_matrix)
#     # print(metrics)
#     # dist2 = euclidean_distances(gt_matrix, metrics)
#     # print("DIST: ", dist2)
#     # dis = pairwise_distances(pts, metric='manhattan'

#     # Calculate the Frobenius norm of the difference
#     dist_eucl = np.linalg.norm(metrics - gt_matrix, 'fro') #'fro' #Frobenius norm  #1 #1-norm #np.inf #infinity-norm
#     print("DIST EUCL: ", dist_eucl)
#     distances.append(dist_eucl)
#     dist_1 = np.linalg.norm(metrics - gt_matrix, 1) #'fro' #Frobenius norm  #1 #1-norm #np.inf #infinity-norm
#     print("DIST 1NORM: ", dist_1)
#     distances.append(dist_1)
#     dist_inf = np.linalg.norm(metrics - gt_matrix, np.inf) #'fro' #Frobenius norm  #1 #1-norm #np.inf #infinity-norm
#     print("DIST INFNORM: ", dist_inf)
#     distances.append(dist_inf)


#     ## FROBENIOS 
#     M_d = np.array([[0.03, 0.03, 0.03], [0.02, 0.04, 0.03], [0.03, 0.03, 0.03]])  # Deformed cloth
#     M_d = M_d.reshape(-1, 1)
#     # Compute the Frobenius norm of the difference
#     D = np.linalg.norm(metrics - gt_matrix, 'fro')

#     # Normalize (optional: to express it as a percentage)
#     D_norm = D / 0.03 #np.linalg.norm(gt_matrix, 'fro') * 100  # Normalize relative to the flat cloth

#     print(np.linalg.norm(gt_matrix-gt_matrix, 'fro'))
#     print("Frobenius norm distance:", D)
#     print("Normalized deformation (%):", D_norm)

#     ## PAIRWISE DISTANCES
#      # Compute pairwise distance matrices
#     D_f = cdist(gt_matrix, gt_matrix)  # Distances in flat cloth
#     D_d = cdist(metrics, metrics)  # Distances in deformed cloth

#     absolute_diff = np.abs(D_d - D_f)
#     # For non-zero distances in the flat cloth, calculate relative change
#     with np.errstate(divide='ignore', invalid='ignore'):
#         relative_change = np.where(D_f > 1e-8, absolute_diff / D_f, absolute_diff)

#     # Mean of relative changes (ignoring infinities and NaNs)
#     mean_relative_change = np.nanmean(relative_change)
    
#     # Scale to percentage
#     deformation_measure = mean_relative_change * 100
#     print(f"Deformation measure: {deformation_measure:.2f}%")

#     return distances

def placing_qual(metrics, n_div, grasp_edge_size, obj_thickness, n_objs):

    min_depth = obj_thickness *n_objs #Object/pile thickness should be 0 deformation
    max_depth = (grasp_edge_size/2) + obj_thickness + 0.01
    half_max_depth = min_depth+0.01 #max_depth/2
    print("min depth: ", min_depth, " / max depth: ", max_depth)

    metrics = np.array(metrics)
    flat_placement = min_depth*np.ones(n_div*n_div)
    # bad_placement = np.array([[0.05, 0.1, 0.05], [0.05, 0.1, 0.05], [0.05, 0.1, 0.05]])
    bad_placement = np.array([[half_max_depth, max_depth, half_max_depth], [half_max_depth, max_depth, half_max_depth], [half_max_depth, max_depth, half_max_depth]]) #to check which is the most representative
    bad_placement = bad_placement.reshape(-1, 1)
    # bad_placement = 0.1*np.ones(n_div*n_div)
    print("Metrics: ", metrics)
    print("GOOD matrix: ", flat_placement)
    print("BAD matrix: ", bad_placement)

    max_dist = np.linalg.norm(bad_placement - flat_placement, 1) #Max distance to perfect placement - Used for normalization
    dist = np.linalg.norm(metrics - flat_placement, 1) #Distance of current sample to perfect placement
    print("Max dist", max_dist)
    print("Dist", dist)
    # placing_error = (dist/max_dist)*100 # Normalize distance
    placing_error = ((dist-min_depth)/(max_dist-min_depth))*100 # Normalize distance
    # placing_error = (dist/0.23)*100 # Normalize distance
    placing_quality = 100-placing_error # Get placing quality (not error)
    print("Placing quality: ", round(placing_quality), "%")

    return placing_quality
    
# def tests():

#     # gt_matrix = 0.03*np.ones(9)
#     # gt_matrix = gt_matrix.reshape(-1, 1) 
#     gt_matrix = np.array([[0.03, 0.03, 0.03], [0.03, 0.03, 0.03], [0.03, 0.03, 0.03]])

#     flat = M_d = np.array([[0.03, 0.03, 0.03], [0.03, 0.03, 0.03], [0.03, 0.03, 0.03]])
#     real_flat = M_d = np.array([[0.03, 0.03, 0.03], [0.02, 0.04, 0.03], [0.03, 0.03, 0.04]])
#     real_deform = np.array([[0.05, 0.11, 0.03], [0.05, 0.2, 0.03], [0.05, 0.1, 0.03]])
#     complete_deform = np.array([[0.15, 0.15, 0.15], [0.15, 0.15, 0.15], [0.15, 0.15, 0.15]])
#     small_deform = np.array([[0.1, 0.07, 0.03], [0.07, 0.04, 0.03], [0.03, 0.03, 0.02]])

#     flat_dists = []
#     real_flat_dists = []
#     real_deform_dists = []
#     complete_deform_dists = []
#     small_deform_dists = []

#     dist_fro = np.linalg.norm(complete_deform - gt_matrix, 'fro') 
#     complete_deform_dists.append(dist_fro)
#     dist_1 = np.linalg.norm(complete_deform - gt_matrix, 1) #'fro' #Frobenius norm  #1 #1-norm #np.inf #infinity-norm
#     complete_deform_dists.append(dist_1)
#     dist_inf = np.linalg.norm(complete_deform - gt_matrix, np.inf) #'fro' #Frobenius norm  #1 #1-norm #np.inf #infinity-norm
#     complete_deform_dists.append(dist_inf)
#     print("MAX DEFORM: ", complete_deform_dists)

#     dist = np.linalg.norm(flat - gt_matrix, 'fro') #'fro' #Frobenius norm  #1 #1-norm #np.inf #infinity-norm
#     dist = (dist/dist_fro)*100
#     flat_dists.append(dist)
#     dist = np.linalg.norm(flat - gt_matrix, 1) #'fro' #Frobenius norm  #1 #1-norm #np.inf #infinity-norm
#     dist = (dist/dist_1)*100
#     flat_dists.append(dist)
#     dist = np.linalg.norm(flat - gt_matrix, np.inf) #'fro' #Frobenius norm  #1 #1-norm #np.inf #infinity-norm
#     dist = (dist/dist_inf)*100
#     flat_dists.append(dist)
#     print("FLAT:", flat_dists)

#     dist = np.linalg.norm(real_flat - gt_matrix, 'fro') #'fro' #Frobenius norm  #1 #1-norm #np.inf #infinity-norm
#     dist = (dist/dist_fro)*100
#     real_flat_dists.append(dist)
#     dist = np.linalg.norm(real_flat - gt_matrix, 1) #'fro' #Frobenius norm  #1 #1-norm #np.inf #infinity-norm
#     dist = (dist/dist_1)*100
#     real_flat_dists.append(dist)
#     dist = np.linalg.norm(real_flat - gt_matrix, np.inf) #'fro' #Frobenius norm  #1 #1-norm #np.inf #infinity-norm
#     dist = (dist/dist_inf)*100
#     real_flat_dists.append(dist)
#     print("REAL FLAT:", real_flat_dists)

#     dist = np.linalg.norm(small_deform - gt_matrix, 'fro') #'fro' #Frobenius norm  #1 #1-norm #np.inf #infinity-norm
#     dist = (dist/dist_fro)*100
#     small_deform_dists.append(dist)
#     dist = np.linalg.norm(small_deform - gt_matrix, 1) #'fro' #Frobenius norm  #1 #1-norm #np.inf #infinity-norm
#     dist = (dist/dist_1)*100
#     small_deform_dists.append(dist)
#     dist = np.linalg.norm(small_deform - gt_matrix, np.inf) #'fro' #Frobenius norm  #1 #1-norm #np.inf #infinity-norm
#     dist = (dist/dist_inf)*100
#     small_deform_dists.append(dist)
#     print("SMALL DEFORM:", small_deform_dists)

#     dist = np.linalg.norm(real_deform - gt_matrix, 'fro') #'fro' #Frobenius norm  #1 #1-norm #np.inf #infinity-norm
#     dist = (dist/dist_fro)*100
#     real_deform_dists.append(dist)
#     dist = np.linalg.norm(real_deform - gt_matrix, 1) #'fro' #Frobenius norm  #1 #1-norm #np.inf #infinity-norm
#     dist = (dist/dist_1)*100
#     real_deform_dists.append(dist)
#     dist = np.linalg.norm(real_deform - gt_matrix, np.inf) #'fro' #Frobenius norm  #1 #1-norm #np.inf #infinity-norm
#     dist = (dist/dist_inf)*100
#     real_deform_dists.append(dist)
#     print("REAL DEFORM:", real_deform_dists)

##################################################################################################
##################################################################################################
## For one unique sample
if not all_files:
    print("\033[94m Getting experiment file: \033[0m" + pcd_file)
    ## ---Get object name---
    for o_name in CLOTH_SIZE:
        if o_name in pcd_file: #Get object name and dims for canonical dimensions
            obj_name = o_name 
            print(pcd_file)
            obj_edge_size = CLOTH_SIZE.get(obj_name, None)
            object_thickness = obj_edge_size[2]
            if(grasped_by=="short"):
                nongrasped_edge_size = obj_edge_size[1]
                grasped_edge_size = obj_edge_size[0] # shortest edge is grasped
            else:
                grasped_edge_size = obj_edge_size[1] # longest edge is grasped
                nongrasped_edge_size = obj_edge_size[0]
            if piling:
                n_objects=2 #Used to multiply object thickness for placing quality (0 deformation)
            else:
                n_objects=1
            ## if piling

    ## ---Process data---
    obj_pcd = o3d.io.read_point_cloud(pcd_dir)
    obj_data = np.asarray(obj_pcd.points)
    filtered_sample = filter_sample(obj_data, raw_sample_filter_box) ##Remove noise points - necessary in placing?
    transl_data, depth_mean = translate_data(filtered_sample, cam_to_table) ##Move points to 0 (from table)
    # norm_transl_data, norm_depth_mean = normalize_transl_data(transl_data)
    # plot_raw_data(obj_data)
    # plot_raw_data(filtered_sample)
    # plot_raw_data(transl_data)
    # plot_raw_data(norm_transl_data)
    # plot(transl_data, "TRANSL", plot_scale, plot_scale_color) ## Plot translated point cloud
    
    ## ---Divide in grids---
    can_x_grid_divs, can_y_grid_divs, can_edges  = create_canonical(n_divisions, gripper_position, grasped_edge_size, nongrasped_edge_size) #get grid divisions
    grids = grid_division(transl_data, can_x_grid_divs, can_y_grid_divs, n_divisions)
    # plot(grids[0], "grid", plot_scale, plot_scale_color)

    ## ---Compute metric---
    mean_metrics = def_metric(grids, grasped_edge_size)
    plot_with_info(transl_data, can_x_grid_divs, can_y_grid_divs, can_edges, pcd_file, plot_scale, plot_scale_color)
    plot_metrics(pcd_file.replace(".pcd", ""), mean_metrics, plot_scale_color)
    # plot_metrics(pcd_file.replace(".pcd", ""), norm_mean_metrics, plot_scale_color)

    ## Compute placing quality computing the distance of the grid metric to the gt metric (0 deformation)
    # distan(mean_metrics, n_divisions)
    # distan(norm_mean_metrics, n_divisions)

    # tests()
    placing_qual(mean_metrics, n_divisions, grasped_edge_size, object_thickness, n_objects)

    

## Process all files in directory
if all_files:
    if(save_csv):
        ## Create CSV file to save metrics
        means_data_file = write_dir + "means_data.csv" ## CSV file to save def metric
        my_file = open(means_data_file, "w")
        means_data_wr = csv.writer(my_file, delimiter=",")
        ##Write CSV headers
        # headers = ["File","Depth mean", "Depth Median"]
        # headers = ["File"]
        # means_data_wr.writerow(headers)

        # ## Create CSV file to save distances to GT
        # dist_data_file = write_dir + "dist_data.csv" ## CSV file to save dist metrics
        # my_file2 = open(dist_data_file, "w")
        # dist_data_wr = csv.writer(my_file2, delimiter=",")
        # ##Write CSV headers
        # headers = ["File","Eucl dist", "1 norm", "inf norm"]
        # dist_data_wr.writerow(headers)

    ## Loop files in directory
    for filename in sorted(os.listdir(data_directory)):
        f = os.path.join(data_directory, filename)
        if os.path.isfile(f) and filename.endswith('.pcd'):
            print("-------------------------------------------------------------------------------")
            ## ---Get object name---
            for o_name in CLOTH_SIZE:
                if o_name in filename:
                    obj_name = o_name
                    print(filename)
                    # if piling
                    
                    ## ---Process data---
                    obj_pcd = o3d.io.read_point_cloud(f)
                    obj_data = np.asarray(obj_pcd.points)
                    filtered_sample = filter_sample(obj_data, raw_sample_filter_box) ##Remove table points
                    transl_data, depth_mean = translate_data(filtered_sample) ##Move points to 0 (from gripper)
                    norm_transl_data, norm_depth_mean = normalize_transl_data(transl_data)
                    # plot(transl_data, "TRANSL", plot_scale, plot_scale_color) ## Plot translated point cloud

                    ## ---Divide in grids---
                    can_x_grid_divs, can_y_grid_divs, can_edges, obj_dimensions = create_canonical(obj_name, n_divisions, gripper_position) #get grid divisions
                    grids = grid_division(norm_transl_data, can_x_grid_divs, can_y_grid_divs, n_divisions)
                    
                    ## ---Compute metric---
                    mean_metrics, norm_mean_metrics = def_metric(grids, obj_dimensions)
                    # plot_metrics(filename.replace(".pcd", ""), mean_metrics, plot_scale_color)

                    ## ---Compute distance between mean matrix (grid) and GT (0 deformation)
                    dist_values = distan(mean_metrics, n_divisions)

                    ##Save data
                    if(save_csv): ##Save means in csv
                        # save_data_values(filename.replace(".pcd", ""), mean_metrics, dist_values)
                        save_data(means_data_wr, filename.replace(".pcd", ""), mean_metrics) #Save mean metrics in csv
                        # plot(norm_transl_data, filename.replace(".pcd", ""), plot_scale, plot_scale_color) ## In /plots
                        # plot_with_info(norm_transl_data, can_x_grid_divs, can_y_grid_divs, can_edges, filename.replace(".pcd", ""), plot_scale, plot_scale_color)
                        # plot_metrics(filename.replace(".pcd", ""), mean_metrics, plot_scale_color) ## In means/

    

## REFS
# Colormap scale in 3D scatter plots: https://plotly.com/python-api-reference/generated/plotly.express.scatter_3d
# Distances: https://www.tutorialspoint.com/python-pairwise-distances-of-n-dimensional-space-array
# https://stackoverflow.com/questions/1401712/how-can-the-euclidean-distance-be-calculated-with-numpy