# Compute deformation cluster with ROS

## SENSE Deformation class with ROS

Provides a ROS Service ``/pick_n_place/sense_def_class`` of type ``SenseDefClass`` that computes the grid metric and clusterizes it. It resturn the deformation cluster where it pertains.
<!-- ## Offline - Without the robot -->

To compute the deformation cluster of a grasped object in real time (through ROS topic), execute:

```
roscore
rosrun pick_n_place main.py 
rosbag play grasped_object_sample.bag
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

## PREDICT Deformation class with ROS

Provides a ROS Service ``/pick_n_place/predict_def_class`` of type ``PredictDefClass`` that trains (or uses an already trained model) to predict deformation class based on categorical and numerical object properties such as layers, grasp, nongraspedsize, graspedsize, area, folded stiffness and friction.

<!-- ## Offline - Without the robot -->
To predict the deformation class of a new sample of object properties using a trained model, execute:

```
roscore
rosrun pick_n_place prediction_module.py
rosservice call /pick_n_place/predict_def_class "{layers: '8l', grasp: 'short', nongraspedsize: 26.0, graspedsize: 25.0, area: 650.0, stiffness: 100.0, friction: 80.0}"
```


