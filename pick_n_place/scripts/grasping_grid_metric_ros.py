## grasping_grid_metric.py to be used as package in ros node (import functions)
## This code measures grid metric of grasped data

## 1. Read PCD file of segmented placed cloth
## 2. Filters pointcloud with a given box to remove outliers
## 3. Translates pointcloud data to move the gripper position points to 0 and the rest to depth negative values
## 4. Normalizes data so the metric is agnostic to the object's size: gripper points correspond to 0 and -1 to half of non-grasped edge size
## 5. Computes grid metric
## In process. Compute grid mean matrix distance to GT matrix (0 deformation) - Used to draw a plot of deformation for each object, fold case and grasp

import numpy as np
import statistics as sts
import plotly.express as px
import plotly.graph_objs as go
import rospy

# colorscale = px.colors.sample_colorscale("jet", [0, 0.7], low=0, high=1)
# colorscale = px.colors.sequential.Jet[int(len(px.colors.sequential.Jet) * 0.3):]
colorscale = px.colors.sample_colorscale("jet", np.linspace(0.1, 0.95, 256))
# colorscale = px.colors.sample_colorscale("hsv_r", np.linspace(0.3, 1, 256))


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

    # if not all_files:
    #     fig.show()
    # if save_csv:
    #     filename = write_dir + file_name + ".jpg"
    #     fig.write_image(filename)
    return fig

## Saves RGB images with the corresponding filename, GT class and metrics
def plot_with_info(data, x_grid_divs, y_grid_divs, can_edges, scale, scale_color):
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
    
    fig = px.scatter_3d(x=data[:,0], y=data[:,1], z=data[:,2], color=data[:,2])#, color_continuous_scale=colorscale)
    fig.update_traces(marker=dict(color="teal"))

    # Plot Canonical plane
    x_can = np.linspace(can_edges[0],can_edges[1],100)
    y_can = np.linspace(can_edges[2],can_edges[3],50)
    z_can = -0.02*np.ones(len(y_data))
    canonic = go.Surface(x=x_can, y=y_can, z=np.array([z_can]*len(x_can)).T, colorscale=bright_pink, opacity=0.2)
    canonic_plane.append(canonic)

    # Plot X axis divisions
    for n in range(1,len(x_grid_divs)-1):
        x=x_grid_divs[n]*np.ones(len(x_can))
        y=np.linspace(min(y_can),max(y_can),100)
        z=np.linspace(min(z_data)-0.01,max(z_data)+0.01,50)
        plane = go.Surface(x=x, y=y, z=np.array([z]*len(x)), colorscale=bright_orange, opacity=0.4)
        planes_x.append(plane)
    # Plot Y axis divisions
    for n in range(1,len(y_grid_divs)-1):
        x=np.linspace(min(x_can),max(x_can),100)
        y=y_grid_divs[n]*np.ones(len(y_can))
        z=np.linspace(min(z_data)-0.01,max(z_data)+0.01,50)
        plane = go.Surface(x=x, y=y, z=np.array([z]*len(x)).T, colorscale=bright_orange, opacity=0.4)
        planes_y.append(plane)

    # fig.add_traces(data)
    fig.add_traces(planes_x)
    fig.add_traces(planes_y)
    fig.add_traces(canonic_plane)
    # fig.update_layout(scene=dict(zaxis=dict(range=[0, 0.2]), xaxis=dict(range=[0.4, 0.05]), yaxis=dict(range=[0.2, -0.2]), aspectratio=dict(x=1, y=1, z=1) ))
    fig.update_layout(scene=dict(zaxis=dict(range=[-1, 0]), xaxis=dict(range=[-0.1, 0.2]), yaxis=dict(range=[0.2, -0.2]), aspectratio=dict(x=1, y=1, z=1) ))
    # fig.update_coloraxes(cmax=0.12, cmin=0.0)
    fig.update_layout(scene=scale)
    fig.update_coloraxes(cmin=scale_color[0], cmax=scale_color[1])
    fig.update_layout( # Increase font sizes
    scene=dict(
        xaxis=dict(title="X Axis", titlefont=dict(size=35), tickfont=dict(size=16)),  # Increase X labels
        yaxis=dict(title="Y Axis", titlefont=dict(size=35), tickfont=dict(size=16)),  # Increase Y labels
        zaxis=dict(title="Z Axis", titlefont=dict(size=35), tickfont=dict(size=16))   # Increase Z labels
    ),
    # font=dict(size=16)  # Increase general font size
    )
    

    return fig

## Plot metrics in colored grid
def plot_metrics(metrics, scale_color):
    metrics = np.array(metrics)
    div=int(np.sqrt(len(metrics)))
    metrics = metrics.reshape(div,div)
    flipped_matrix = metrics[::-1, ::-1] # Flip both rows and columns to see it in another perspective (from the front of the robot)
    # print("Metrics: ", metrics)
    # colorscale = px.colors.sample_colorscale("Viridis", [start, end])
    fig = px.imshow(flipped_matrix, text_auto=True, color_continuous_scale=colorscale) #text_auto=True, labels=dict(x='x', y='y'))
    fig.update_coloraxes(cmin=scale_color[0], cmax=scale_color[1])#cmax=0.08, cmin=0.0)
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
    
    scale = dict(zaxis=dict(range=[z_min, z_max]), xaxis=dict(range=[x_min, x_max]), yaxis=dict(range=[y_min, y_max]) )
    scale_color = [z_min, z_max]
    fig = plot(data, "raw", scale, scale_color)

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

## Moves pointcloud to have gripper points to 0
def translate_data(obj_data, cam_to_gripper):
    ## Traslate depth (0(gripper)-deformation)
    depth = obj_data[:,0]
    transl_data = []
    not_pile_data = []
    suma = 0
    for i in range(len(depth)):
        point = cam_to_gripper - depth[i] #point that will be 0
        suma += point
        new_point=[obj_data[i,2], obj_data[i,1], point] #changed axis to have z as depth
        transl_data.append(new_point)

    transl_data = np.array(transl_data)
    transl_depth = transl_data[:,2]
    # transl_depth = np.array(transl_data)[:,2]
    mean = sts.mean(transl_depth)
    median = sts.median(transl_depth)
    metrics = [mean, median]

    # activate_print=True
    # print_info(activate_print, "Global Mean: ", mean)
    # print_info(activate_print, "Global Median: ", median)
    # print_info(activate_print, "Y min: ", min(obj_data[:,1]))
    # print_info(activate_print, "Y max: ", max(obj_data[:,1]))
    # print_info(activate_print, "Edge Y: ", max(obj_data[:,1])-min(obj_data[:,1]))
    # print_info(activate_print, "X min: ", min(obj_data[:,2]))
    # print_info(activate_print, "X max: ", max(obj_data[:,2]))
    # print_info(activate_print, "Edge X: ", max(obj_data[:,2])-min(obj_data[:,2]))
    # activate_print=False

    return transl_data, metrics

## Normalizes pointcloud, where 0 is gripper position and -1 is non-grasped edge size
def normalize_transl_data(transl_data, non_grasped_edge_size):
    # obj_edge_size = CLOTH_SIZE.get(obj_name, None) #Get object edges size
    # max_depth = obj_edge_size[non_grasped_edge]
    max_depth = non_grasped_edge_size
    depth = transl_data[:,2]
    norm_transl_data = []
    for i in range(len(depth)):
        point = depth[i]/max_depth
        new_point=[transl_data[i,0], transl_data[i,1], point]
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
def create_canonical(grasped_edge_size, non_grasped_edge_size, n_div, gripper_position):
    # print("\033[96m Creating canonical for ", obj_name, " \033[0m")
    rospy.loginfo("Deformation_clustering: Creating canonical")

    xmin = xmax = ymin = ymax = 0
    x_thrs = []
    y_thrs = []

    # obj_edge_size = CLOTH_SIZE.get(obj_name, None)
    
    # ymin = gripper_position[1]-(obj_edge_size[grasped_edge]/2)
    # ymax = gripper_position[1]+(obj_edge_size[grasped_edge]/2)
    # xmax = gripper_position[0] 
    # xmin = gripper_position[0]-obj_edge_size[non_grasped_edge]
    ymin = gripper_position[1]-(grasped_edge_size/2)
    ymax = gripper_position[1]+(grasped_edge_size/2)
    xmax = gripper_position[0] 
    xmin = gripper_position[0]-non_grasped_edge_size

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

    # # print("OBJ DIMS: ", obj_edge_size)
    # print_info(activate_print, xmin, " / ", xmax, " / ", ymin, " / ", ymax)
    # print_info(activate_print, x_thrs, " / ", y_thrs)

    return x_thrs, y_thrs, canonical_edges#, obj_edge_size

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
            grids.append(grid2)

    # print_info(activate_print,"Data size: ", len(data))
    # for i in range(0,len(grids)):
    #     print_info(activate_print,"Grid size: ", len(grids[i]))
    #     print_info(activate_print,"Data size: ", len(data))
    #     for i in range(0,len(grids)):
    #         print_info(activate_print,"Grid size: ", len(grids[i]))

    return grids

## Computes mean of each grid section
def def_metric(grids):

    means = []
    ## For each section of the grid
    for l in range (len(grids)):
        length = len(grids[l])
        # print_info(activate_print, "\033[94m Grid length \033[0m", length)
        ## If there are no points in the grid, then the mean is max deformation
        if(length == 0):
            means.append(-1)
        ## If the grid is not empty, compute mean of depth
        else:
            depth = grids[l][:,2]
            new_grid = grids[l]
            grid_mean = sts.mean(depth)
            # print_info(activate_print, "Grid mean: ", grid_mean)
            means.append(grid_mean)
    
    # print("Means: ", means)

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

