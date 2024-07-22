## THIS CODE MEASURES MEAN AND MEDIAN OF DEPTH
## GOAL: 
## 1. Read PCD file of segmented placed cloth
## 2. Measure placing quality (golab mean/median of depth, grid metric, finddd?)
## 3. Do a table of the different plans cost
import numpy as np
import plotly.express as px
import open3d as o3d
import os
import csv
import statistics as sts


all_files = False
data_directory ="/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/PCD_placing/"
pcd_file = "p_se_v_1.pcd" #t_se_v_1.pcd"
pcd_dir = data_directory+pcd_file
write_dir = "/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/plots/"
save_csv = True

cam_to_table = 0.8
obj_thickness = 0.02
# min_def = cam_to_table-obj_thickness # Minimum deformation (0 deformation)

##################################################################################################
## UTIL FUNCTIONS
def plot(data, file_name):
    data = np.array(data)
    fig = px.scatter_3d(x=data[:,0], y=data[:,1], z=data[:,2], color=data[:,2])
    #plotly.offline.plot({"data": [fig1], "layout": mylayout}, auto_open=True)
    #fig.update_layout(scene=dict(zaxis=dict(range=[0.8, 0.5]), xaxis=dict(range=[0.3, 0]), yaxis=dict(range=[0.2, -0.2]) ))
    fig.update_layout(scene=dict(zaxis=dict(range=[0, 0.2]), xaxis=dict(range=[0.3, 0.05]), yaxis=dict(range=[0.2, -0.2]), aspectratio=dict(x=1, y=1, z=1) ))
    fig.update_coloraxes(cmax=0.12, cmin=0.0)
    # fig.show()
    filename = write_dir + file_name + ".jpg"
    fig.write_image(filename)

def save_data_values(exp_name, data_values):
    print("\033[94m Writing deformation metric values... \033[0m")
    data = []
    data.append(exp_name)
    for i in range(len(data_values)):
        data.append(data_values[i])
    # data.append(data_values)
    means_data_wr.writerow(data)

## DATA PROCESS FUNCTIONS
def translate_data(obj_data):
    ## Traslate depth (0-object height)
    depth = obj_data[:,0]
    transl_data = []
    suma = 0
    for i in range(len(depth)):
        #point = (depth[i]-can_min_depth)/(1-can_min_depth)
        point = cam_to_table - depth[i]
        suma += point
        new_point=[obj_data[i,2], obj_data[i,1], point]
        transl_data.append(new_point)

    transl_depth = np.array(transl_data)[:,2]
    mean = sts.mean(transl_depth)
    median = sts.median(transl_depth)
    # print("Deformation: ", mean-obj_thickness)
    metrics = [mean, median]

    return transl_data, metrics

##################################################################################################
## MAIN

## For one unique sample
if not all_files:
    print("\033[94m Getting experiment file: \033[0m" + pcd_file)
    ## Get sample point cloud
    obj_pcd = o3d.io.read_point_cloud(pcd_dir)
    obj_data = np.asarray(obj_pcd.points)
    # plot(obj_data, "EXPERIMENT") ## Plot sample point cloud
    transl_data, depth_mean = translate_data(obj_data)
    plot(transl_data, "TRANSL") ## Plot translated point cloud
    

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
            print(filename)
            obj_pcd = o3d.io.read_point_cloud(f)
            obj_data = np.asarray(obj_pcd.points)
            transl_data, depth_mean = translate_data(obj_data)
            plot(transl_data, filename) ## Plot translated point cloud

            ##Save data
            if(save_csv): ##Save means in csv
                save_data_values(filename.replace(".pcd", ""), depth_mean)

    
## OK- "Normalize/Translate" depth data: Put points at the table level as 0 and heigher points >0
## OK (on notion) - Then, "normalize" deformation: Points underneath object thickness are OK (def=0), 
## points over object thickness correspond to deformation (def>0). What do we consider def=1?

## REFS
# Colormap scale in 3D scatter plots: https://plotly.com/python-api-reference/generated/plotly.express.scatter_3d
