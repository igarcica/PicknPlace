Copyright (C) 2009-2010 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
Author Irene Garcia-Camacho (igarcia@iri.upc.edu).

# PICK GARMENT DEMONSTRATION

This package was used to execute the experiments for the RA-L article "Benchmakring bimanual cloth manipulation" with Tiago robot from PAL Robotics. Performs the first part of spreading a tablecloth task: Grasp a corner of a crumpled or folded tablecloth placed on a table, offer it to a second robot which will grasp the second corner performing edge tracing, and spread it using its mobile platform. It is intended to be used with the package "tiago_pcl_tutorial" for locating the garment's corner. In addition, for performing the whole bimanual task, the second robot has to run the package "grament_sliding".

## Getting started:

The project must have the following structure inside iri-lab:

- labrobotica/
  - *algorithms/*
    - *iriutils*
- iri_ws/
  - *src/*
    - *iri_core*
    - **pick_garment_demo**
    - **tiago_pcl_tutorial**

## Dependencies

Compile the packages with:

``catkin build -j2 tiago_pcl_tutorial``
``catkin build pick_garment_demo``

After making a catkin build the following error may appear:

``fatal error: pal_interaction_msgs/TtsAction.h: No such file or directory``

if so, execute the following command, where <pkg> is the name of the pkg that gives error (e.g. pal_interaction_msgs):

``sudo apt install pal-erbium-<pkg>-dev``

It is also needed to install iriutils on labrobotica:

```sh
cd ~/iri-lab/labrobotica/algorithms/iriutils
cd build
cmake ..
make
sudo make install
```

## Setup

In order to perform motion planning using obstacle avoidance, the octomap generation must be enabled on Tiago in the file *move_group.launch* located at the *tiago_moveit_config* package:

```
ssh pal@tiago-72c
su
rw
chroot /ro
vim tiago_moveit_config/launch/move_group.launch
Change camera bool to true
exit
ro
reboot
```

*Note: Remember to revert the changes once finalised all the tests.*

This demo is pretended to be used on Tiago robot. To do so, all the necessary packages must be deployed on the robot with:

```
rosrun pal_deploy deploy.py -p <package> tiago-72c
```
The necessary packages to deploy are:

- pick_garment_demo
- tiago_modules
- tiago_pcl_tutorial
- iri_ros_tool

It is also necessary to add and install iriutils inside Tiago in the following path /home/pal/iri-lab/labrobotica/algorithms/iriutils and add its lib path to the env variable $LD_LIBRARY_PATH

`` export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/pal/iri-lab/labrobotica/algorithms/iriutils ``
`` sudo ldconfig ``

## Execution in real robot:

Once the software has been deployed, run the launch file in the robot:

```
ssh tiago-72c
roslaunch pick_garment_demo pick_garment_demo_tiago.launch
```

To visualize the generated Octomap, the pointclouds, and the grasping marker, run on the development computer:

```
roslaunch pick_garment_demo pick_garment_demo_dev.launch
```

The rqt_reconfigure includes the following variables:

- ***Start SM:***
  - **start**: Starts the state machine from initial state [cr] or [fd].
  - **start_give**: Starts state machine from initial state [pg1].
  - **start_platform**: Starts state machine from [pg2].
  - **go**: Continues to the following phase.
  - **stop**: Stops the state machine.
- ***Configuration parameters***:
  - **pregrasp_offset_x**: Offset in m with respect the grasping position for the pregrasp position (same for y and z).
  - **postgrasp_offset_x**: Offset in m with respect the grasping position for the postgrasp position (same for y and z).
  - **roll, pitch** and **yaw**: Arm orientation of the grasping pose.
  - **position_tol** and **orientation_tol**: Tolerance for the arm position and orientation during grasp.
- ***Test pose parameters:***
  - **test**: Starts the state machine from initial state but for grasping the given position.
  - **frame_id**: Reference frame of the fiven position.
  - **x_pos**, **y_pos** and **z_pos**: Cartesian position of the grasping target.


