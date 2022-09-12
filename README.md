Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
Author Irene Garcia-Camacho (igarcia@iri.upc.edu).

# Demo Pick and Place

This package is used to perform Pick & Place of folded cloths. It can be used with cloths of different size, thickness and rigidity. Grasps larger edge of the cloth and based on its deformation after picked it places it using different placing trajectories. It can also be used to place in piles.

## Getting started

The package has the following structure:

- pick_n_place: Contains the state machine to perform the pick and place.
- vision_pick_place: Contains all the necessary code related to perception (Segmentation, corner detection, grasp point selection, pile height, etc)
- iri_kinova_linear_movement: For execution cartesian movements with Kinova.

## Execution

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
