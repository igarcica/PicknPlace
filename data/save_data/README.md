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
		- complete_grasp_data/: Data (PCD, RGB, RVIZ, rosbag) for 6 objects (towel, pillowcase, checkered, waffle, cotton nap, linen nap), 5 folds (4l, 6l, 8l, 12l, 16l) and 4 grasps (long me, long se, short me, short se)
		- complete_grasp_data_PCD/: Only PCD files of complete_grasp_data
		- complete_grasp_data_metric/: Results from executing '/placing_metric/grasping_grid_metric.py' and with complete_grasp_data. Metric color images and csv with metric for different grid divisions
			- AxA/metric: Metric color images and CSV with metrics
			- AxA/clusters: Metric images separated by clusters
		- traintest_complete_grasp_data/: PCD files separated into train (towel, pillowcase, waffle and cotton napkin) and test (checkered and linen napkin)
		- def/, PCD_placing/, placing_metric/: (to delete) previous data
		- system_adaptability: Data of executions to show how the placement quality improves and the costs are updated based on the results. Piles of 2 objects. Towels and pillowcases used.
		- system_scalability: Data of executions for proving the capability of the system to build piles of more objects (piles of 4). Towel
		- system_performance: Data of executions for comparing a naive system (state machine grasping nearest edge + placing vertically) and whole system (planning with prediction and sensing). Several objects with several foldings


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

### Deformation metrics

To save the metric color images and the CSV with the metrics into /complete_grasp_data_metric/AxA/metric, modify the grid division (n_divisions) and execute:

```
cd data/placing_metric
python3 grasping_grid_metric.py
```

To clusterize all the data (without separating into train/test), execute:

```
python3 clustering_raw.py
```

This will save the metric color images into separated folders for each cluster in /complete_grasp_data_metric/AxA/clusters.

To clusterize separating into train/test data (obtain kmeans model with train and predict cluster of test data), change directory in grasping_grid_metric.py to /traintest_complete_grasp_data/train or test and execute:

```
python3 clustering_raw_traintest.py
```

### Using ROS nodes

Check README from /pick_n_place/scripts for estimating the deformation class and placing quality of the grasped cloth and placed cloth data, using the ROS nodes used in the demo. 


