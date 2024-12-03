## This code measures grid metric of grasped data

## 1. Read PCD file of segmented placed cloth
## 2. Filters pointcloud with a given box to remove outliers
## 3. Translates pointcloud data to move the gripper position points to 0 and the rest to depth negative values
## 4. Normalizes data so the metric is agnostic to the object's size: gripper points correspond to 0 and -1 to half of non-grasped edge size
## 5. Computes grid metric
## In process. Compute grid mean matrix distance to GT matrix (0 deformation) - Used to draw a plot of deformation for each object, fold case and grasp

import numpy as np
import os
import csv
import open3d as o3d
import statistics as sts
import plotly.express as px
import plotly.graph_objs as go


all_files = False
# data_directory ="/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/grasping_data/PCD_grasping_folds/"
data_directory ="/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/complete_grasp_data_PCD/"
pcd_file = "towel_8l_long_me.pcd" #cotnap_6l_long_me.pcd" #"towel_12l_short_se.pcd" #waffle_12l_long_me.pcd"
# pcd_file = "towel_62l_se.pcd"
pcd_dir = data_directory+pcd_file
write_dir = "/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/complete_grasp_data_metric/train_test/3x3/metric/"

save_csv = False
activate_print = False

n_divisions = 3
cam_to_gripper = 0.35 ## Used in transl_data to define Minimum deformation (gripper point)
gripper_position = [0.12, -0.025] ## Used to compute grid divisions

raw_sample_filter_box = [[0.3, 0.7], [-0.2, 0.2], [-0.3, 0.3]] #box to filter sample removing noise points
plot_scale = dict(xaxis=dict(range=[0.2, -0.2]), yaxis=dict(range=[0.2, -0.2]), zaxis=dict(range=[-1, 0]), aspectratio=dict(x=1, y=1, z=1) ) #plot scale for grasped samples
plot_scale_color = [-1, 0.0] # plot depth color scale for grasped samples

CLOTH_SIZE = {
    "towel_2l": (0.45,0.5),
    "towel_4l": (0.25,0.45),
    "towel_6l": (0.25, 0.3),
    # "towel_61l": (0.17,0.46),
    # "towel_62l": (0.27,0.32),#0.32 0.27 or viceversa?
    "towel_8l": (0.23,0.25),
    "towel_12l": (0.15,0.25),
    "pillowc_2l": (0.44,0.54),
    "pillowc_4l": (0.28,0.45),
    "pillowc_6l": (0.23,0.37),
    # "pillowc_61l": (0.19,0.46),
    # "pillowc_62l": (0.23,0.47),
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
non_grasped_edge = 0 
grasped_edge = 1 #longest edge is grasped

##################################################################################################
## UTIL FUNCTIONS

def print_info(activate, arg1, arg2="", arg3="", arg4="", arg5="", arg6="", arg7=""):
    if(activate):
        print(str(arg1) + str(arg2) + str(arg3) + str(arg4) + str(arg5) + str(arg6) + str(arg7))

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
    z=-0.2*np.ones(len(y_data))
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

## Moves pointcloud to have gripper points to 0
def translate_data(obj_data):
    ## Traslate depth (0(gripper)-deformation)
    depth = obj_data[:,0]
    transl_data = []
    not_pile_data = []
    suma = 0
    for i in range(len(depth)):
        #point = (depth[i]-can_min_depth)/(1-can_min_depth)
        point = cam_to_gripper - depth[i] #point that will be 0
        suma += point
        new_point=[obj_data[i,2], obj_data[i,1], point] #changed axis to have z as depth
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
    print_info(activate_print, "Y min: ", min(obj_data[:,1]))
    print_info(activate_print, "Y max: ", max(obj_data[:,1]))
    print_info(activate_print, "Edge Y: ", max(obj_data[:,1])-min(obj_data[:,1]))
    print_info(activate_print, "X min: ", min(obj_data[:,2]))
    print_info(activate_print, "X max: ", max(obj_data[:,2]))
    print_info(activate_print, "Edge X: ", max(obj_data[:,2])-min(obj_data[:,2]))
    activate_print=False

    return transl_data, metrics

## Normalizes pointcloud, where 0 is gripper position and -1 is non-grasped edge size
def normalize_transl_data(transl_data):
    obj_edge_size = CLOTH_SIZE.get(obj_name, None) #Get object edges size
    max_depth = obj_edge_size[non_grasped_edge]
    depth = transl_data[:,2]
    norm_transl_data = []
    for i in range(len(depth)):
        point = depth[i]/max_depth
        new_point=[obj_data[i,2], obj_data[i,1], point]
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
def create_canonical(obj_name, n_div, gripper_position):
    print("\033[96m Creating canonical for ", obj_name, " \033[0m")

    xmin = xmax = ymin = ymax = 0
    x_thrs = []
    y_thrs = []

    obj_edge_size = CLOTH_SIZE.get(obj_name, None)
    
    ymin = gripper_position[1]-(obj_edge_size[grasped_edge]/2)
    ymax = gripper_position[1]+(obj_edge_size[grasped_edge]/2)
    xmax = gripper_position[0] 
    xmin = gripper_position[0]-obj_edge_size[non_grasped_edge]

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

    print("OBJ DIMS: ", obj_edge_size)
    print_info(activate_print, xmin, " / ", xmax, " / ", ymin, " / ", ymax)
    print_info(activate_print, x_thrs, " / ", y_thrs)

    return x_thrs, y_thrs, canonical_edges, obj_edge_size

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
def def_metric(grids, obj_edge_size):

    means = []
    ## For each section of the grid
    for l in range (len(grids)):
        length = len(grids[l])
        print_info(activate_print, "\033[94m Grid length \033[0m", length)
        ## If there are no points in the grid, then the mean is max deformation
        if(length == 0):
            # means.append(-obj_edge_size[non_grasped_edge]+0.05/2) #Max depth (should be 1 when normalized). +5cm to give margin
            means.append(-1)
        ## If the grid is not empty, compute mean of depth
        else:
            depth = grids[l][:,2]
            new_grid = grids[l]
            grid_mean = sts.mean(depth)
            print_info(activate_print, "Grid mean: ", grid_mean)
            means.append(grid_mean)
    
    print("Means: ", means)

    return means

def distan(metrics, n_div):
    distances = []

    metrics = np.array(metrics)
    metrics = metrics.reshape(-1, 1)
    # hola = listofzeros = [0] * n_div*n_div
    # print(hola)
    gt_matrix = np.zeros(n_div*n_div)
    gt_matrix = gt_matrix.reshape(-1, 1) 
    # print(type(metrics))
    # print(type(hola))
    # print(gt_matrix)
    # print(metrics)
    # dist2 = euclidean_distances(gt_matrix, metrics)
    # print("DIST: ", dist2)
    # dis = pairwise_distances(pts, metric='manhattan'

    # Calculate the Frobenius norm of the difference
    dist_eucl = np.linalg.norm(metrics - gt_matrix, 'fro') #'fro' #Frobenius norm  #1 #1-norm #np.inf #infinity-norm
    print("DIST: ", dist_eucl)
    distances.append(dist_eucl)
    dist_1 = np.linalg.norm(metrics - gt_matrix, 1) #'fro' #Frobenius norm  #1 #1-norm #np.inf #infinity-norm
    print("DIST: ", dist_1)
    distances.append(dist_1)
    dist_inf = np.linalg.norm(metrics - gt_matrix, np.inf) #'fro' #Frobenius norm  #1 #1-norm #np.inf #infinity-norm
    print("DIST: ", dist_inf)
    distances.append(dist_inf)

    print(distances)

    return distances


##################################################################################################
##################################################################################################
## For one unique sample
if not all_files:
    print("\033[94m Getting experiment file: \033[0m" + pcd_file)
    ## ---Get object name---
    for o_name in CLOTH_SIZE:
        if o_name in pcd_file: #Get object name for canonical dimensions
            obj_name = o_name 
            print(pcd_file)
            if "long" in pcd_file: #get grasped and non grasped edge positions from CLOTH_SIZE
                non_grasped_edge = 0 
                grasped_edge = 1 #longest edge is grasped
            if "short" in pcd_file: #get grasped and non grasped edge positions from CLOTH_SIZE
                non_grasped_edge = 1 
                grasped_edge = 0 #shortest edge is grasped
    ## ---Process data---
    obj_pcd = o3d.io.read_point_cloud(pcd_dir)
    obj_data = np.asarray(obj_pcd.points)
    filtered_sample = filter_sample(obj_data, raw_sample_filter_box) ##Remove table points
    transl_data, depth_mean = translate_data(filtered_sample) ##Move points to 0 (from gripper)
    norm_transl_data, norm_depth_mean = normalize_transl_data(transl_data)
    # plot_raw_data(obj_data)
    # plot_raw_data(filtered_sample)
    # plot_raw_data(transl_data)
    # plot_raw_data(norm_transl_data)
    plot(norm_transl_data, "TRANSL", plot_scale, plot_scale_color) ## Plot translated point cloud
    
    ## ---Divide in grids---
    can_x_grid_divs, can_y_grid_divs, can_edges, obj_edge_size  = create_canonical(obj_name, n_divisions, gripper_position) #get grid divisions
    grids = grid_division(norm_transl_data, can_x_grid_divs, can_y_grid_divs, n_divisions)
    # plot(grids[0], "grid", plot_scale, plot_scale_color)

    ## ---Compute metric---
    mean_metrics = def_metric(grids, obj_edge_size)
    plot_with_info(norm_transl_data, can_x_grid_divs, can_y_grid_divs, can_edges, pcd_file, plot_scale, plot_scale_color)
    plot_metrics(pcd_file.replace(".pcd", ""), mean_metrics, plot_scale_color)

    distan(mean_metrics, n_divisions)

    

## Process all files in directory
if all_files:
    if(save_csv):
        ## Create CSV file to save metrics
        means_data_file = write_dir + "all_metrics.csv" ## CSV file to save def metric
        my_file = open(means_data_file, "w")
        means_data_wr = csv.writer(my_file, delimiter=",")
        ## Write Headers
        headers = ["Filename"]
        for i in range(0, n_divisions*n_divisions):
                text = "M"+str(i+1)
                headers.append(text)
        means_data_wr.writerow(headers)
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
                    if "long" in filename: #get grasped and non grasped edge positions from CLOTH_SIZE
                        non_grasped_edge = 0 
                        grasped_edge = 1 #longest edge is grasped
                    if "short" in filename:
                        non_grasped_edge = 1 
                        grasped_edge = 0 #shortest edge is grasped
                    
                    ## ---Process data---
                    obj_pcd = o3d.io.read_point_cloud(f)
                    obj_data = np.asarray(obj_pcd.points)
                    filtered_sample = filter_sample(obj_data, raw_sample_filter_box) ##Remove table points
                    transl_data, depth_mean = translate_data(filtered_sample) ##Move points to 0 (from gripper)
                    norm_transl_data, norm_depth_mean = normalize_transl_data(transl_data)
                    # plot(transl_data, "TRANSL", plot_scale, plot_scale_color) ## Plot translated point cloud

                    ## ---Divide in grids---
                    can_x_grid_divs, can_y_grid_divs, can_edges, obj_edge_size = create_canonical(obj_name, n_divisions, gripper_position) #get grid divisions
                    grids = grid_division(norm_transl_data, can_x_grid_divs, can_y_grid_divs, n_divisions)
                    
                    ## ---Compute metric---
                    mean_metrics = def_metric(grids, obj_edge_size)
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

    
## OK -Normalize metric (from 0 to max depth (non-grasped edge size))

## REFS
# Colormap scale in 3D scatter plots: https://plotly.com/python-api-reference/generated/plotly.express.scatter_3d
# Distances: https://www.tutorialspoint.com/python-pairwise-distances-of-n-dimensional-space-array
# https://stackoverflow.com/questions/1401712/how-can-the-euclidean-distance-be-calculated-with-numpy