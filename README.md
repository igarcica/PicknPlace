Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
Author Irene Garcia-Camacho (igarcia@iri.upc.edu).

# Demo Pick and Place

This package is used to perform Pick & Place of folded cloths. It can be used with cloths of different size, thickness and rigidity. Grasps larger edge of the cloth and based on its deformation after picked it places it using different placing trajectories. It can also be used to place in piles.

Version used for getting data (color png images, depth topics rosbags and pointloud pcd) with 3 cameras from pick and place executions.

## Getting started

The package has the following structure:

- pick_n_place: Contains the state machine to perform the pick and place.
- vision_pick_place: Contains all the necessary code related to perception (Segmentation, corner detection, grasp point selection, pile height, etc)
- iri_kinova_linear_movement: For execution cartesian movements with Kinova.
- data: Scripts to save data from pick and place executions.

<!--## How to execute PnP demo

First launch the camera node and robot driver, in this example the rs camera and kinova robot:
Launch the camera and the kortex driver:

``roslaunch pick_n_place camera_n_kinova.launch``

Launch the nodes corresponding to the demo (iri_kinova_linear_movement, pick_n_place and vision_pick_place):

``roslaunch pick_n_place picknplace_demo.launch``

This will launch the RVIZ to visualize the perception system and rqt reconfigure to control the demo, which includes the following variables:

The rqt_reconfigure includes the following variables:

- ***Start SM:***
  - **get_grasp_point**: Confirm the grasp point selected (pink point in RVIZ). 
  - **start**: Starts the state machine.
  - **go**: Continues with the placing execution after checking the deformation.
  - **stop**: Stops the state machine.
  - **close_gripper**: gripper closing parameter (1.0 is completely close)
- ***Configuration parameters***:
  - **handeye**: XYZ and RPY offsets for handeye transformation between camera and kinova base.
- ***Test pose parameters:***
  - **test**: Starts the state machine from initial state but for grasping the given position.
  - **frame_id**: Reference frame of the fiven position.
  - **grasp**: Grasping target pose for testing.
-->

## How to save PnP executions

Launch the 3 cameras (zenithal, frontal and lateral). It can be in either way each camera separately:

``roslaunch realsense2_camera rs_camera.launch filters:=pointcloud serial_no:=024222250058 camera:=ext_camera

roslaunch realsense2_camera rs_camera.launch filters:=pointcloud serial_no:=846112071003 camera:=frontal_camera

roslaunch realsense2_camera rs_camera.launch filters:=pointcloud serial_no:=031222070182 camera:=lateral_camera``

or together with the launch file:

``roslaunch pick_n_place cameras.launch``

Launch the kinova driver:

``roslaunch pick_n_place kinova.launch``

Launch the demo:

``roslaunch pick_n_place picknplace_demo.launch``


To save the executions, go to the destination folder and run the script to generate inner folders:

``cd PnP_data
./create_folders``

To save color png and depth pcd, go to the appropiate folder (grasp, place, place_grasp) and run the script to save color pngs, rosbags of the depth topics and pointloud pcd:

``cd PnP/ExpX/grasp
../../commands.sh``


To visualize the saved data, you must publish the pointcloud saved in the pcd file as:

``rosrun pcl_ros pcd_to_pointcloud pointcloud.pcd 0.1 (_frame_id:=/ext_camera_link)``

and visualize it either using rviz or pcl viewer:

``rosrun rviz rviz -d visualize.rviz``
``pcl_viewer filename.pcd``


### How to extract deformation from data

Publish the point cloud of the pcd file:

``rosrun pcl_ros pcd_to_pointcloud pointcloud.pcd 0.1``

Run the vision node:

``roslaunch vision_pick_place picknplace.launch``
