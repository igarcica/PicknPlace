Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
Author Irene Garcia-Camacho (igarcia@iri.upc.edu).

# Demo Pick and Place

This package is used to perform Pick & Place of folded cloths. It can be used with cloths of different size, thickness and rigidity. Grasps larger edge of the cloth and based on its deformation after picked it places it using different placing trajectories. It can also be used to place in piles.

Version used for getting data (color png images, depth topics rosbags and pointloud pcd) with 3 cameras from pick and place executions.

## Getting started

The package has the following structure:

- **pick_n_place**: Contains the state machine to perform the pick and place.
- **vision_pick_place**: Contains all the necessary code related to perception (Segmentation, corner detection, grasp point selection, pile height, etc)
- **iri_kinova_linear_movement**: For execution cartesian movements with Kinova.
- **pnp_planner**: Includes the PDDL model and launch files to generate the plan and send it to ROS system.
- **ROSPlan**: Package to link PDDL model to ROS system.
- **data**: Scripts to save data from pick and place executions and compute deformation metric.
    - */save_data*: Scripts to save data form pick and place executions (Color and depth images, depth topis in rosbag and pointcloud data as pcd file).
    - */save_metric*: 
        - /c: Contains the code (in C++) to compute and write csv files with the deformation data (using the pcd files with full view and vision node)
            - execution.py: Publishes the content of PCD files as point cloud during 8seg one at a time.
            - /node: ROS node that subscribes to topic with deformation data (computed and published by the vision_pick_place node) and writes CSV files.
        - /ptyhon: Contains the code (in python) to compute and write the csv files with the deformation data (using pcd files with segmented garment).
            - def_metric.py: Computes the deformation metric and saves it in CSV files.
            - clustering.py: Computes the centroid and inter/intra distances of the deformation data from csv files.
            - kmeans.py: Clusterises pick files based on deformation metrics and computes success rate comparing it to the ground truth.
            - plot_results.py: Prints the deformation metrics in the corresponding files for visual information.

<!--## Dependencies

For hierarchhical.py

sudp apt get insttal python3-pip
pip3 install -U scikit-learn
pip3 install numpy, pandas, matplotlib
-->

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
