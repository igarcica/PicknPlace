<!-- # Compute deformation cluster with ROS -->
# Getting started

This folder contains python scripts for the modules necessary for the system (prediction and state estimation modules) as well as some useful scripts for plotting and updating the planning costs. The relevant files are:

- prediction_module.py: ROS node with the service to predict the deformation class given the object properties and grasp location. Loads the trained model in ``/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/placing_metric/random_forest_model.pkl``
- sense_def_class.py: ROS node with the service to sense the current deformation class. Uses ``grasping_grid_metric_ros.py`` and ``clustering_raw_traintest_ros.py``
    - grasping_grid_metric_ros.py: Contains the functions to process the pointcloud of the grasped cloth and obtain the grid-based metric.
    - clustering_raw_traintest_ros.py: Contains the functions to classify the current grid-based metric in a deformation class. Loads the trained model in ``/home/userlab/iri-lab/iri_ws/src/PicknPlace/data/placing_metric/kmeans_model.pkl``
- placing_quality.py: ROS node with the service to compute the quality of the placed object or pile. Uses ``placing_grid_metric_ros.py``
    - placing_grid_metric_ros.py: Contains the functions to process the pointcloud of the placed object and compute the metric for the placing quality.
- cost_update.py: Given the placement qualities, placing strategies and sensed deformation classes, it computes and plots the costs tables.
    - system_adaptability.py: Plots the pile quality and cost evolution of the system's adaptabilty experiments.
    - stystem_performance.py
    - system_scalability.py
    - compute_cost_entry.py: ROS node with a service to compute the new state-action cost entry given a placement quality. 



#  Deformation class prediction, Deformation state estimation and Placing state estimation modules

## PREDICT Deformation class with ROS

Provides a ROS Service ``/pick_n_place/predict_def_class`` of type ``PredictDefClass`` that trains (or uses an already trained model) to predict deformation class based on categorical and numerical object properties such as layers, grasp, nongraspedsize, graspedsize, area, folded stiffness and friction.

<!-- ## Offline - Without the robot -->
To predict the deformation class of a new sample of object properties using a trained model, execute:

```
roscore
rosrun pick_n_place prediction_module.py
rosservice call /pick_n_place/predict_def_class "{layers: '8l', grasp: 'short', nongraspedsize: 26.0, graspedsize: 25.0, area: 650.0, stiffness: 100.0, friction: 80.0}"
```

## SENSE Deformation class with ROS

Provides a ROS Service ``/pick_n_place/sense_def_class`` of type ``SenseDefClass`` that computes the grid metric and clusterizes it. It returns the deformation cluster where it pertains.
<!-- ## Offline - Without the robot -->

To compute the deformation cluster of a grasped object in real time (through ROS topic), execute:

```
roscore
rosrun pick_n_place sense_def_class.py 
rosservice call /pick_n_place/get_deformation_class
```

When a point cloud message is published in the topic /segment_table/place, the node will process the point cloud to compute the grid metric and clusterize it.
It can be run without the robot with ``rosbag play grasped_object_sample.bag``

<!-- ## Online - With the robot

Launch the robot and camers, the pick and place demo and the deformation clustering node:

```
roslaunch pick_n_place camera_n_kinova.launch
roslaunch pick_n_place picknplace_demo.launch
rosrun pick_n_place main.py 
```

Start the demo with the rqt_reconfigure. When the object arrives to the CHOOSE_PLACING state, it will call the service and obtain a deformation class -->


## Estimate Placing quality

Provides a ROS Service ``/pick_n_place/get_placing_quality`` of type ``GetPlacingQual`` that provides the placing quality of the placed or piled object using the grid metric. It requires the object name (for the object dimensions of the canonical to create the grid division), the grasped edge (to orientate the canonical) and wether the evaluation is of a pile or not (to know the expected minimum depth).

To measure the placing quality in real time (through ROS topic), execute:

```
roscore
rosrun pick_n_place placing_quality.py 
rosservice call /pick_n_place/get_placing_quality "object_name: 'towel' grasped_edge: 'short' pile: false"
```

When a point cloud message is published in the topic /segment_table/place, the node will process the point cloud to compute the grid metric and compute the placing quality.

It can be run without the robot with ``rosbag play placed_object_sample.bag`` or with PCD files by changing the topic to /cloud_pcd and running ``rosrun pcl_ros pcd_to_pointcloud filename.pcd 0.1``



## Update planning cost tables

<!--Define the current cost table for placing and piling. Define the placing strategy, sensed deformation class and resulting placing error of the new observation with the parameters placing_strategy_pile, def_class_pile and placing_error_pile, respectively. Run the script to obtain the updated cost tables:

 ```
python3 placing_cost_update.py
``` -->

Given the placing quality history, sensed deformation class and placing strategy, it computes the cost tables and plots the placing quality and costs evolution:

<!--```
python3 cost_update.py
``` -->
```
python3 system_adaptability.py
```

To compute the new state-action cost given a placement quality (through ROS topic), execute:

```
roscore
rosrun pick_n_place compute_cost_entry.py 
rosservice call /pick_n_place/compute_cost_entry "cost_table: [17, 1, 1, 8, 22, 6, 30, 25, 6], def_class: 'A' placing_str: 'placevert' placing_qual: 90"
```


<!-- # PLOT FIGURES

## Plot placing quality and cost update

Given the placing quality history, sensed deformation class and placing strategy, it computes the cost update tables and plots the placing quality and costs evolution:
``python3 main_cost_update.py`` -->



<!-- Para borrar?:
python3 plot_placing_quality.py
python3 placing_cost_update.py -->