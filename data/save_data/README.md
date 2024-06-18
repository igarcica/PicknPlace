Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
Author Irene Garcia-Camacho (igarcia@iri.upc.edu).

# Saving PnP data

These scripts include commands to save data from the PnP demo (pick_n_place package) such as color and depth images as well as pointclouds of the deformed grasped or placed cloth.  

Version used for getting data (color png images, depth topics rosbags and pointloud pcd) with 3 cameras from pick and place executions.

## Getting started

This folder contains the following files:

- data/:
	- **save_data/**:
		- commands.sh: Saves 1 rgb image from each of the 3 cameras, saves depth topic rosbags and 1 pcd files for each camera.
		- create_folders.sh: Creates folder for saving data of the cloth grasped, placed while grasping and placed.
		- visualize.rviz: RVIZ configure file for visualizing the data saved in the pcd files.


## How to save PnP executions

Launch the 3 cameras (zenithal, frontal and lateral). It can be in either way each camera separately:

```
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud serial_no:=024222250058 camera:=ext_camera

roslaunch realsense2_camera rs_camera.launch filters:=pointcloud serial_no:=846112071003 camera:=frontal_camera

roslaunch realsense2_camera rs_camera.launch filters:=pointcloud serial_no:=031222070182 camera:=lateral_camera
```

or together with the launch file (serial numbers and names can be modified):

``roslaunch pick_n_place multiple_cameras.launch``

Launch the kinova driver and the demo if :

``roslaunch pick_n_place kinova.launch``

Launch the demo:

``roslaunch pick_n_place picknplace_demo.launch``


To save the executions, go to the destination folder and run the script to generate inner folders:

```
cd PnP_data
./create_folders
```

To save color png and depth pcd, go to the appropiate folder (grasp, place, place_grasp) and run the script to save color pngs, rosbags of the depth topics and pointloud pcd:

```
cd PnP/ExpX/grasp
../../commands.sh
```


To visualize the saved data, you must publish the pointcloud saved in the pcd file as:

``rosrun pcl_ros pcd_to_pointcloud pointcloud.pcd 0.1 (_frame_id:=/ext_camera_link)``

and visualize it either using rviz or pcl viewer:

```
rosrun rviz rviz -d visualize.rviz
pcl_viewer filename.pcd
```


### How to extract deformation from data (to update with clustering package)

To visualize the computed metrics publish the point cloud of the pcd file and run the vision node:

```
roscore
rosrun pcl_ros pcd_to_pointcloud pointcloud_file.pcd 0.1
rostopic echo /segment_table/corners
roslaunch vision_pick_place picknplace.launch
```

To run all the pcd files one by one and write the results in a csv file:

```
roscore
rosrun save_metric_csv save_metric_csv.py
roslaunch vision_pick_place picknplace.launch
cd data_folder
python execution.py
```

This will
