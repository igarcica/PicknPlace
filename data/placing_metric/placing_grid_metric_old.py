## Code to compute placing metric. Old version of grasping_grid_metric.py, update it using the plot scale and transl_ used here.
import numpy as np
import os
import csv
import open3d as o3d
import statistics as sts
from sklearn.metrics.pairwise import euclidean_distances
# from scipy.spatial.distance import cdist
import plotly.express as px
import plotly.graph_objs as go


all_files = False
data_directory ="/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/placing_data/PCD_placing/"
pcd_file = "towel_me_r_1.pcd" #towel_me_d_1.pcd" #towel_se_v_2.pcd" #p_se_v_1.pcd" #t_se_v_1.pcd"
# data_directory ="/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/PCD_grasping_folds/"
# pcd_file = "towel_8l_me.pcd"
pcd_dir = data_directory+pcd_file
write_dir = "/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/def/3x3_range/"

save_csv = False
activate_print = False

cam_to_table = 0.8
# min_def = cam_to_table-obj_thickness # Minimum deformation (0 deformation)
objects = ["towel", "checkered", "pillowc"]
n_divisions = 3

plot_scale = dict(zaxis=dict(range=[0, 0.2]), xaxis=dict(range=[0.4, 0.05]), yaxis=dict(range=[0.2, -0.2]), aspectratio=dict(x=1, y=1, z=1) ) #placing
# plot_scale = dict(zaxis=dict(range=[0.8, 0.5]), xaxis=dict(range=[0.3, 0]), yaxis=dict(range=[0.2, -0.2]) )

##################################################################################################
## UTIL FUNCTIONS

def print_info(activate, arg1, arg2="", arg3="", arg4="", arg5="", arg6=""):
    if(activate):
        print(str(arg1) + str(arg2) + str(arg3) + str(arg4) + str(arg5) + str(arg6))


def plot(data, file_name, scale):
    data = np.array(data)
    fig = px.scatter_3d(x=data[:,0], y=data[:,1], z=data[:,2], color=data[:,2])
    ##plotly.offline.plot({"data": [fig1], "layout": mylayout}, auto_open=True)
    ##fig.update_layout(scene=dict(zaxis=dict(range=[0.8, 0.5]), xaxis=dict(range=[0.3, 0]), yaxis=dict(range=[0.2, -0.2]) ))
    fig.update_layout(scene=scale)
    fig.update_coloraxes(cmax=0.12, cmin=0.0)
    if not all_files:
        fig.show()
    else:
        filename = write_dir + file_name + ".jpg"
        fig.write_image(filename)

## Saves RGB images with the corresponding filename, GT class and metrics
def plot_with_info(can, data, x_grid_divs, y_grid_divs, filename):
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

    # can = go.Scatter3d(x=can[:,0], y=can[:,1], z=can[:,2])
    # fig.add_traces(data)
    fig.add_traces(planes_x)
    fig.add_traces(planes_y)
    fig.update_layout(scene=dict(zaxis=dict(range=[0, 0.2]), xaxis=dict(range=[0.4, 0.05]), yaxis=dict(range=[0.2, -0.2]), aspectratio=dict(x=1, y=1, z=1) ))
    fig.update_coloraxes(cmax=0.12, cmin=0.0)
    if not all_files:
        fig.show()
    else:
        filename = write_dir + filename + ".jpg"
        fig.write_image(filename)

def plot_metrics(filename, metrics):
    metrics = np.array(metrics)
    div=int(np.sqrt(len(metrics)))
    metrics = metrics.reshape(div,div)
    fig = px.imshow(metrics, text_auto=True, labels=dict(x='x', y='y'))
    fig.update_coloraxes(cmax=0.08, cmin=0.0)

    if not all_files:
        fig.show()
    else:
        filename = write_dir + filename + ".jpg"
        fig.write_image(filename)

def plot_raw_data(data):
    x_min = min(data[:,0])
    x_max = max(data[:,0])
    y_min = min(data[:,1])
    y_max = max(data[:,1])
    z_min = min(data[:,2])
    z_max = max(data[:,2])
    print("xmin: ", x_min, "xmax: ", x_max)
    print("ymin: ", y_min, "ymax: ", y_max)
    print("zmin: ", z_min, "zmax: ", z_max)
    scale = dict(zaxis=dict(range=[z_min, z_max]), xaxis=dict(range=[x_min, x_max]), yaxis=dict(range=[y_min, y_max]), aspectratio=dict(x=1, y=1, z=1) )
    plot(data, "raw", scale)

def save_data_values(exp_name, data_values):
    print("\033[94m Writing deformation metric values... \033[0m")
    data = []
    data.append(exp_name)
    for i in range(len(data_values)):
        data.append(data_values[i])
    # data.append(data_values)
    means_data_wr.writerow(data)


## DATA PROCESS FUNCTIONS
## Moves pointcloud data from cam_to_table to 0-...
def translate_data(obj_data, obj_thickness):
    ## Traslate depth (0-object height)
    depth = obj_data[:,0]
    transl_data = []
    not_pile_data = []
    suma = 0
    for i in range(len(depth)):
        #point = (depth[i]-can_min_depth)/(1-can_min_depth)
        point = cam_to_table - depth[i]
        suma += point
        new_point=[obj_data[i,2], obj_data[i,1], point]
        transl_data.append(new_point)

        if piling:
            if point > obj_thickness:
                not_pile_data.append(new_point)
    
    not_pile_data = np.array(not_pile_data)

    transl_data = np.array(transl_data)
    transl_depth = transl_data[:,2]
    # transl_depth = np.array(transl_data)[:,2]
    mean = sts.mean(transl_depth)
    median = sts.median(transl_depth)
    print("Mean: ", mean)
    print("Median: ", median)
    # print("Deformation: ", mean-obj_thickness)
    metrics = [mean, median]

    print_info(activate_print, "Y min: ", min(obj_data[:,1]))
    print_info(activate_print, "Y max: ", max(obj_data[:,1]))
    print_info(activate_print, "Edge Y: ", max(obj_data[:,1])-min(obj_data[:,1]))
    print_info(activate_print, "X min: ", min(obj_data[:,2]))
    print_info(activate_print, "X max: ", max(obj_data[:,2]))
    print_info(activate_print, "Edge X: ", max(obj_data[:,2])-min(obj_data[:,2]))

    return transl_data, metrics, not_pile_data

def create_canonical(obj_name, n_div):

    syn_can_matrix = []
    syn_can_x = []
    syn_can_y = []
    syn_can_depth = []
    print_info(activate_print, obj_name)
    xmin = xmax = ymin = ymax = 0
    x_thrs = []
    y_thrs = []

    if(obj_name == "towel"): #towel
        print("\033[96m Creating canonical for towel... \033[0m")
        xsteps = 0.009
        xmin = 0.09
        xmax = xmin+0.25 #grasped towel edge
        ysteps = 0.01
        ymin = -0.15
        ymax = ymin+0.25
        obj_thickness = 0.04
    if(obj_name == "pillowc"): #pillowc
        print("\033[96m Creating canonical for pillowcase... \033[0m")
        xsteps = 0.009
        xmin = 0.09
        xmax = xmin+0.28 #grasped towel edge
        ysteps = 0.01
        ymin = -0.15
        ymax = ymin+0.23
        obj_thickness = 0.013
    if(obj_name == "checkered"): #checkerec
        print("\033[96m Creating canonical for checkered rag... \033[0m")
        xsteps = 0.009
        xmin = 0.09
        xmax = xmin+0.26 #grasped towel edge
        ysteps = 0.01
        ymin = -0.15
        ymax = ymin+0.2
        obj_thickness= 0.02

    x_thr = (xmax - xmin)/n_div
    y_thr = (ymax - ymin)/n_div

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

    return x_thrs, y_thrs, obj_thickness

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

def def_metric(grids, obj_thickness):

    means = []
    def_metrics = []
    ## For each section of the grid
    for l in range (len(grids)):
        length = len(grids[l])
        print_info(activate_print, "\033[94m Grid length \033[0m", length)
        depth = grids[l][:,2]
        new_grid = grids[l]
        grid_mean = sts.mean(depth)
        print_info(activate_print, "Grid mean: ", grid_mean)
        means.append(grid_mean)

        if piling:
            grid_def = grid_mean-(obj_thickness*2)
        else:
            grid_def = grid_mean-obj_thickness
        def_metrics.append(grid_def) 
    
    print("Means: ", means)
    print("Def metrics: ", def_metrics)

    return means, def_metrics

def distan(metrics, n_div):
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
    dist2 = euclidean_distances(gt_matrix, metrics)
    print("DIST: ", dist2)
    # dis = pairwise_distances(pts, metric='manhattan'


##################################################################################################
##################################################################################################
## For one unique sample
if not all_files:
    print("\033[94m Getting experiment file: \033[0m" + pcd_file)
    ## Get object name
    for n in range(len(objects)):
        if objects[n] in pcd_file:
            obj_name = objects[n]
            if "2" in pcd_file:
                piling = True
            else:
                piling = False
    # print()
    ## Get sample point cloud
    obj_pcd = o3d.io.read_point_cloud(pcd_dir)
    obj_data = np.asarray(obj_pcd.points)
    plot_raw_data(obj_data)
    can_x_grid_divs, can_y_grid_divs, obj_thick = create_canonical(obj_name, n_divisions)
    transl_data, depth_mean, not_pile_data = translate_data(obj_data, obj_thick)
    plot(transl_data, "TRANSL", plot_scale) ## Plot translated point cloud
    if piling:
        print("plot pile")
        # plot(not_pile_data, "Not pile", plot_scale) ## Plot sample of pile without bottom cloth
        transl_data = not_pile_data ## For piling samples use pointcloud with removed piling points

    ## Process point cloud
    grids = grid_division(transl_data, can_x_grid_divs, can_y_grid_divs, n_divisions)
    mean_metrics, def_metrics = def_metric(grids, obj_thick)
    plot_with_info(transl_data, transl_data, can_x_grid_divs, can_y_grid_divs, pcd_file)
    plot_metrics(pcd_file.replace(".pcd", ""), def_metrics)

    distan(def_metrics, n_divisions)

    

## Process all files in directory
if all_files:
    if(save_csv):
        ## Create CSV file to save metrics
        means_data_file = write_dir + "means_data.csv" ## CSV file to save def metric
        my_file = open(means_data_file, "w")
        means_data_wr = csv.writer(my_file, delimiter=",")
        ##Write CSV headers
        headers = ["File","Depth mean", "Depth Median"]
        means_data_wr.writerow(headers)

    ## Loop files in directory
    for filename in sorted(os.listdir(data_directory)):
        f = os.path.join(data_directory, filename)
        if os.path.isfile(f) and filename.endswith('.pcd'):
            print("-------------------------------------------------------------------------------")
            ## Get object name
            for n in range(len(objects)):
                if objects[n] in filename:
                    obj_name = objects[n]
                    print(filename)
                    if "2" in filename:
                        piling = True
                        print("Remove pile points!")
                    else:
                        piling = False

                    obj_pcd = o3d.io.read_point_cloud(f)
                    obj_data = np.asarray(obj_pcd.points)
                    can_x_grid_divs, can_y_grid_divs, obj_thick = create_canonical(obj_name, n_divisions)
                    transl_data, depth_mean, not_pile_data = translate_data(obj_data, obj_thick)
                    # plot(transl_data, filename) ## Plot translated point cloud
                    # plot(not_pile_data, filename)
                    if piling:
                        print("plot pile")
                        # plot(not_pile_data, "Not pile") ## Plot sample of pile without bottom cloth
                        transl_data = not_pile_data ## For piling samples use pointcloud with removed piling points

                    ## Process point cloud
                    
                    grids = grid_division(transl_data, can_x_grid_divs, can_y_grid_divs, n_divisions)
                    mean_metrics, def_metrics = def_metric(grids, obj_thick)
                    # plot_with_info(transl_data, transl_data, can_x_grid_divs, can_y_grid_divs, pcd_file)

                    ##Save data
                    if(save_csv): ##Save means in csv
                        save_data_values(filename.replace(".pcd", ""), def_metrics)
                        plot_with_info(transl_data, transl_data, can_x_grid_divs, can_y_grid_divs, filename.replace(".pcd", ""))
                        plot_metrics(filename.replace(".pcd", ""), def_metrics)

    
## OK- "Normalize/Translate" depth data: Put points at the table level as 0 and heigher points >0
## OK (on notion) - Then, "normalize" deformation: Points underneath object thickness are OK (def=0), 
## points over object thickness correspond to deformation (def>0). What do we consider def=1?

## REFS
# Colormap scale in 3D scatter plots: https://plotly.com/python-api-reference/generated/plotly.express.scatter_3d
# Distances: https://www.tutorialspoint.com/python-pairwise-distances-of-n-dimensional-space-array
# https://stackoverflow.com/questions/1401712/how-can-the-euclidean-distance-be-calculated-with-numpy