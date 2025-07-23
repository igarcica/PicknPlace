#!/usr/bin/env python3

import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from pick_n_place.srv import SenseDefClass, SenseDefClassResponse

import numpy as np
import open3d as o3d
import plotly.express as px

import os, sys
# sys.path.insert(1, "/".join(os.path.realpath(__file__).split("/")[0:-3]))
# from data.placing_metric.grasping_grid_metric import *
import grasping_grid_metric_ros as grid_metric
import clustering_raw_traintest_ros as clustering

import joblib

##################################################################################################
## PARAMS

############ GRID METRIC ############
n_divisions = 3
cam_to_gripper = 0.35 ## Used in transl_data to define Minimum deformation (gripper point)
# gripper_position = [0.151, -0.025] ## Used to compute grid divisions
gripper_position = [0.13, -0.025] ## Used to compute grid divisions

raw_sample_filter_box = [[0.3, 0.7], [-0.2, 0.2], [-0.3, 0.3]] #box to filter sample removing noise points
plot_scale = dict(xaxis=dict(range=[0.2, -0.2]), yaxis=dict(range=[0.2, -0.2]), zaxis=dict(range=[-1, 0]), aspectratio=dict(x=1, y=1, z=1) ) #plot scale for grasped samples
plot_scale_color = [-1, 0.0] # plot depth color scale for grasped samples

CLOTH_SIZE = {
    "towel_2l": (0.45,0.5),
    "towel_4l": (0.25,0.45),
    "towel_6l": (0.25, 0.3),
    "towel_8l": (0.23,0.25),
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
    "waffle_16l": (0.13,0.18),
    "linrag_4l": (0.25,0.35),
    "linrag_6l": (0.24,0.25),
    "linrag_8l": (0.18,0.25, 0.005),
    "linrag_12l": (0.12,0.25),
    "linrag_16l": (0.13,0.18),
    "twlrag_4l": (0.25,0.35),
    "twlrag_6l": (0.24,0.25),
    "twlrag_8l": (0.18,0.25, 0.02),
    "twlrag_12l": (0.12,0.25),
    "twlrag_16l": (0.13,0.18)
    }
# obj_name = "pillowc_8l"
# #default grasped and non-grasped value positions from CLOTH_SIZE
# non_grasped_edge = 0 
# grasped_edge = 1 #longest edge is grasped

# obj_edge_size = CLOTH_SIZE.get(obj_name, None)
# non_grasped_edge_size = obj_edge_size[non_grasped_edge]
# grasped_edge_size = obj_edge_size[grasped_edge]

show_imgs = False
save_data = False

write_dir = "/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/experiments_placing_update/"

############ CLUSTERING ############
n_clusters = 3
# grid_div = 3
# directory = "/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/complete_grasp_data_metric/train_test_3objs/"
# train_data_dir = directory + str(n_divisions) + "x" + str(n_divisions) + "/metric/train_metrics.csv"


##################################################################################################
## UTIL FUNCTIONS

def show_save_figs(figure, name):
    if show_imgs:
        figure.show()
    if save_data:
        filename = write_dir + name + ".jpg"
        figure.write_image(filename)

def say_bye():
    print("Deformation_clustering: Shutting down")

##################################################################################################
## ROS FUNCTIONS

def load_clustering_model():

    global_dir = "/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/placing_metric/"
    clustering_model_dir = global_dir + "kmeans_model.pkl"

    # Load the saved model
    clustering_model_loaded = joblib.load(clustering_model_dir)
    rospy.loginfo("Deformation_clustering: Model loaded successfully")

    return clustering_model_loaded

## Get the grid metrics of the sample - Calls functions from grasping_grid_metric
def process_pointcloud(data, grasp_edge_size, non_grasp_edge_size):

    rospy.loginfo("Deformation_clustering: Received pointcloud message")

    ## Read topic and obtain pointcloud
    cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(data) # Convert PointCloud2 to a numpy structured array
    obj_data = np.stack((cloud_array['x'], cloud_array['y'], cloud_array['z']), axis=-1) # Extract 'x', 'y', and 'z' fields
    obj_data = np.array(obj_data)
    fig = grid_metric.plot_raw_data(obj_data)
    # show_save_figs(fig)
    
    ## ---Process data---
    filtered_sample = grid_metric.filter_sample(obj_data, raw_sample_filter_box) ## Remove table points
    transl_data, depth_mean = grid_metric.translate_data(filtered_sample, cam_to_gripper) ## Move points to 0 (from gripper)
    norm_transl_data, norm_depth_mean = grid_metric.normalize_transl_data(transl_data, non_grasp_edge_size) ## Normalize points from 0 to -1 (max possible depth corresponding to non grasped edge sice)
    fig2 = grid_metric.plot(norm_transl_data, "Processed_pointcloud", plot_scale, plot_scale_color) ## Plot translated point cloud
    # show_save_figs(fig2)
    
    ## ---Divide in grids---
    can_x_grid_divs, can_y_grid_divs, can_edges  = grid_metric.create_canonical(grasp_edge_size, non_grasp_edge_size, n_divisions, gripper_position) #get grid divisions
    grids = grid_metric.grid_division(norm_transl_data, can_x_grid_divs, can_y_grid_divs, n_divisions)
    fig3 = grid_metric.plot_with_info(norm_transl_data, can_x_grid_divs, can_y_grid_divs, can_edges, plot_scale, plot_scale_color)
    show_save_figs(fig3, "grasp_plot")

    ## ---Compute metric---
    mean_metrics = grid_metric.def_metric(grids)
    fig4 = grid_metric.plot_metrics(mean_metrics, plot_scale_color)
    show_save_figs(fig4, "grasp_metric")

    mean_metrics = np.array(mean_metrics)

    return mean_metrics

def handle_service(req):
  
    msg = rospy.wait_for_message('/segment_table/place', PointCloud2) # Get next message from the topis /segment_table/place (segmented pointcloud of the grasped object)
    # msg = rospy.wait_for_message('/cloud_pcd', PointCloud2)
    
    ## ---Get object dimensions for creating canonical---
    object_layers = req.object_name + "_" + req.layers
    obj_edge_size = CLOTH_SIZE.get(object_layers, None)
    # object_thickness = obj_edge_size[2]
    if(req.grasped_edge=="short"):
        grasped_edge_size = obj_edge_size[0] # shortest edge is grasped
        nongrasped_edge_size = obj_edge_size[1]
    else:
        grasped_edge_size = obj_edge_size[1] # longest edge is grasped
        nongrasped_edge_size = obj_edge_size[0]
        
    ## ---Obtain grid metric---
    grid_metric = process_pointcloud(msg, grasped_edge_size, nongrasped_edge_size) 

    # def_class = clusterize_data(grid_metric, train_data_dir, n_clusters, n_divisions) 
    ## ---Predict deformation cluster of current sample---
    grid_metric = grid_metric.reshape(1, -1) # Contains a single sample with n_div*n_div features
    predicted_label = kmeans_model.predict(grid_metric) 
    print("Sensed def class: ", predicted_label)

    int_def_class = predicted_label.item()
    if(int_def_class == 0): #If class is 0 then send "A", etc
        str_def_class = "B"
    elif(int_def_class == 1):
        str_def_class = "A"
    elif(int_def_class == 2):
        str_def_class = "C"

    return SenseDefClassResponse(str_def_class)

if __name__ == '__main__':
    rospy.init_node('sense_def_class', anonymous=True)
    rospy.loginfo("Sense_def_class: Node ready")
    s = rospy.Service('/pick_n_place/sense_def_class', SenseDefClass, handle_service)
    kmeans_model = load_clustering_model()
    rospy.spin()



# #####
# ROS Node to sense the deformation class through the pointcloud topic.
# Description: It works as a service that the SM calls when the PDDL plan action "check_deformation" occurs
# Input: Grasped object pointcloud through topic /segment_table/place
# Output: Deformation cluster class

## USAGE:
# roscore
# rosrun pick_n_place main.py
# rosbag play grasped_object_sample.bag

### To Do
# OK-Process JUST one pointcloud message and quit the node
# OK-Add clustering
# Ensure that the created clustering labels correspond to my labels (i.e. 0 for A class quasi flat deformation)- How?
# Check grid x position to do the division properly
# OK- Integrate with demo (new state in SM, launch file, tc)
# kmeans should be trained considering 9 separated features (in 3x3) or as 3 features (each horizonatl row of grid)?
# Receive from service object dimensions for grid division


### REFS
## https://sentry.io/answers/import-files-from-a-different-folder-in-python/