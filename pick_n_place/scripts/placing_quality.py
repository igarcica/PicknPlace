#!/usr/bin/env python3
import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2
from pick_n_place.srv import GetPlacingQual, GetPlacingQualResponse
import placing_grid_metric_ros as placing_grid_metric
import placing_grid_metric_ros2 as placing_grid_metric2
import numpy as np

n_divisions = 3
cam_to_table = 0.8 ## Used to move points from table (0) to deformation (>0)
# piling = False

raw_sample_filter_box = [[0, cam_to_table], [-0.2, 0.2], [0, 0.3]] # Box to filter sample removing noise points wrt camera axis
# gripper_position = [0.32, -0.025] ## Used to compute grid divisions (wrt ext_camera_link changing x-axis for z-axis)
# gripper_position = [0.27, -0.07]
gripper_position = [0.32, -0.04]
# gripper_position = [0.31, -0.025]

## In the case of the placing metric, the thickness of the objects plays a role
CLOTH_SIZE = {
    "towel": (0.23,0.25, 0.04),
    "pillowc": (0.23,0.28, 0.01),
    "towel_2l": (0.45,0.5),
    "towel_4l": (0.25,0.45),
    "towel_6l": (0.25, 0.3),
    "towel_8l": (0.23,0.25, 0.04), #0.05?
    "towel_12l": (0.15,0.25),
    "pillowc_2l": (0.44,0.54),
    "pillowc_4l": (0.28,0.45),
    "pillowc_6l": (0.23,0.37),
    "pillowc_8l": (0.23,0.28, 0.01),
    "pillowc_12l": (0.15,0.28),
    "pillowc_16l": (0.14,0.23),
    "cotnap_2l": (0.25,0.5),
    "cotnap_4l": (0.25,0.25, 0.01),
    "cotnap_6l": (0.17,0.25),
    "cotnap_12l": (0.09,0.25),
    "cotnap_8l": (0.13, 0.27, 0.006),
    "cotnap_16l": (0.13,0.13),
    "linenap_2l": (0.25,0.5),
    "linenap_4l": (0.25,0.25),
    "linenap_6l": (0.17,0.25),
    "linenap_8l": (0.13,0.25, 0.02),
    "linenap_12l": (0.09,0.25),
    "linenap_16l": (0.13,0.13, 0.02),
    "check_4l": (0.25,0.35),
    "check_6l": (0.24,0.25, 0.01),
    # "check_8l": (0.23,0.27, 0.01),
    "check_8l": (0.18,0.25, 0.01),
    "check_12l": (0.12,0.25),
    "check_16l": (0.13,0.18),
    "waffle_4l": (0.25,0.35),
    "waffle_6l": (0.24,0.25),
    "waffle_8l": (0.18,0.25, 0.017),
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

show_imgs = False
save_imgs = False
write_dir = "/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/save_data/exps_slides/"
plot_scale = dict(xaxis=dict(range=[0, 0.4]), yaxis=dict(range=[0.2, -0.3]), zaxis=dict(range=[0, 0.3]), aspectratio=dict(x=1, y=1, z=1) ) #plot scale for grasped samples
plot_scale_color = [0.0, 0.1] # plot depth color scale for grasped samples


# save_csv = False
# activate_print = False

# plot_scale = dict(xaxis=dict(range=[0, 0.4]), yaxis=dict(range=[0.2, -0.2]), zaxis=dict(range=[0, 0.3]), aspectratio=dict(x=1, y=1, z=1) ) #plot scale for grasped samples
# # plot_scale_color = [0.0, 0.1] # plot depth color scale for grasped samples
# plot_scale_color = [0.0, 1] # plot depth color scale for grasped samples

def show_save_figs(figure, name):
    if show_imgs:
        figure.show()
    if save_imgs:
        filename = write_dir + name + ".jpg"
        figure.write_image(filename)


def process_pointcloud(data, grasp_edge_size, nongrasp_edge_size, obj_thickness, n_objs, pile_thickn):

    rospy.loginfo("Placing_quality: Received pointcloud message")

    # ---Read and process pointcloud---
    cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(data) # Convert PointCloud2 to a numpy structured array
    obj_data = np.stack((cloud_array['x'], cloud_array['y'], cloud_array['z']), axis=-1) # Extract 'x', 'y', and 'z' fields
    obj_data = np.array(obj_data)

    ## ---Process data---
    filtered_sample = placing_grid_metric.filter_sample(obj_data, raw_sample_filter_box) ##Remove noise points - necessary in placing?
    transl_data, depth_mean = placing_grid_metric.translate_data(filtered_sample, cam_to_table) ##Move points to 0 (from table)
    
    ## ---Divide in grids---
    can_x_grid_divs, can_y_grid_divs, can_edges  = placing_grid_metric.create_canonical(n_divisions, gripper_position, grasp_edge_size, nongrasp_edge_size) #get grid divisions
    grids = placing_grid_metric.grid_division(transl_data, can_x_grid_divs, can_y_grid_divs, n_divisions)

    ## ---Compute metric---
    mean_metrics = placing_grid_metric.def_metric(grids, grasp_edge_size, obj_thickness)

    ## ---Plot---
    # figx = placing_grid_metric.plot_raw_data(transl_data)
    # show_save_figs(figx, "Raw")
    fig = placing_grid_metric.plot_with_info(transl_data, can_x_grid_divs, can_y_grid_divs, can_edges, obj_thickness, n_objs, pile_thickn, plot_scale, plot_scale_color)
    show_save_figs(fig, "placing_plot")
    # fig2 = placing_grid_metric.plot_metrics(mean_metrics, plot_scale_color)
    fig2 = placing_grid_metric.plot_metrics(mean_metrics, obj_thickness, n_objs)
    show_save_figs(fig2, "placing_metric")
    
    return mean_metrics

def handle_service(req):
    
    msg = rospy.wait_for_message('/segment_table/place', PointCloud2) # Get next message from the topic /segment_table/place (segmented pointcloud of the placed object)
    # msg = rospy.wait_for_message('/cloud_pcd', PointCloud2) # Get next message from the topic /segment_table/place (segmented pointcloud of the placed object)
    
    ## ---Get object dimensions for creating canonical---
    object_layers = req.object_name + "_" + req.layers
    obj_edge_size = CLOTH_SIZE.get(object_layers, None)
    # obj_edge_size = CLOTH_SIZE.get(req.object_name, None)
    object_thickness = obj_edge_size[2]
    # if(req.n_obj_pile):
    #     n_objects = 2 #If piling, thickness will be multiplied by 2 (or more)
    #     # object_thickness = obj_edge_size[2]*2 #Thickness is to two objects
    # else:
    #     n_objects = 1
    #     # object_thickness = obj_edge_size[2] # If it is first object placed
    n_objects = req.n_objs_pile #The pile quality depends on the number of objects in the pile (including the currently placed object), which assigns the pile thickness/height

    pile_thickness = req.expected_pile_thickn

    if(req.grasped_edge=="short"):
        nongrasped_edge_size = obj_edge_size[1]
        grasped_edge_size = obj_edge_size[0] # shortest edge is grasped
    else:
        grasped_edge_size = obj_edge_size[1] # longest edge is grasped
        nongrasped_edge_size = obj_edge_size[0]

    grid_metric = process_pointcloud(msg, grasped_edge_size, nongrasped_edge_size, object_thickness, n_objects, pile_thickness) # Grid metric

    placing_quality = placing_grid_metric.placing_qual(grid_metric, n_divisions, nongrasped_edge_size, object_thickness, n_objects) # Placing quality
    placing_quality = round(placing_quality)
    placing_quality = np.where(placing_quality < 0, 0, np.where(placing_quality>100, 100, placing_quality))
    print("placing quality:", placing_quality)
    print("Placing error:", 100-placing_quality)

    # placing_quality2 = placing_grid_metric2.compute_quality(grid_metric, n_divisions, object_thickness)
    # print("RESULT: ", placing_quality2)

    # placing_quality3 = placing_grid_metric2.hybrid(grid_metric, n_divisions, object_thickness, n_objects)
    # print("RESULT: ", placing_quality3)
    
    placing_quality_pile = placing_grid_metric.dif_objs_placing_qual(grid_metric, n_divisions, nongrasped_edge_size, pile_thickness, n_objects) # Placing quality
    placing_quality_pile = round(placing_quality_pile)
    placing_quality_pile = np.where(placing_quality_pile < 0, 0, np.where(placing_quality_pile>100, 100, placing_quality_pile))
    print("PILE placing quality:", placing_quality_pile)
    print("PILE Placing error:", 100-placing_quality_pile)
    
    return GetPlacingQualResponse(placing_quality_pile)

if __name__ == '__main__':
    rospy.init_node('placing_quality', anonymous=True)
    rospy.loginfo("Placing_quality: Node ready")
    s = rospy.Service('/pick_n_place/get_placing_quality', GetPlacingQual, handle_service)
    rospy.spin()

### INFO
## ROS node for sensing placing quality
## Provides a ROS Service to measure the placing quality given a pointcloud using the grid metric

### TO DO
## Receive from service request: object dimensions (grasped_edge_size, nongrasped_edge_size, obj_thickness), pile_thickness
## Future work: Use the sensed edge sizes and object thickness
