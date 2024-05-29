Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
Author Irene Garcia-Camacho (igarcia@iri.upc.edu).

# Demo Pick and Place

This package is used to perform Pick & Place of folded cloths. It can be used with cloths of different size, thickness and rigidity. Grasps larger edge of the cloth and based on its deformation after picked it places it using different placing trajectories. It can also be used to place in piles.

## Getting started

Packages necessary for the demo:

- pick_n_place: Contains the state machine to perform the pick and place.
- vision_pick_place: Contains all the necessary code related to perception (Segmentation, corner detection, grasp point selection, pile height, etc)
- iri_kinova_linear_movement: For execution cartesian movements with Kinova.

## Features

- Segments cloth in the table, detects closest largest edge and locates middle point to grasp.
- Grasps the middle point according to its orientation (4 possible grasp orientations).
- moves the grasped cloth under the camera to detect deformation.
- 3 placings strategies can be choosen to place the object (vertically, diagonally and rotating the arm). Dynamic placings can also be performed but executed separately.

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
  - **start_demo**: Starts the state machine.
  - **start_experiments**: Starts the state machine from the placing state to obtain data.
  - **stop**: Stops the state machine.
  - **ok**: Continues with the placing execution after checking the deformation.
  - **close**: Closes the gripper at the placing position (for start_experiments).
  - **open**: Opens gripper once placed.
  - **close_gripper**: Percentage of gripper closing (for different garment thickness).
  - **towel**: "Fast button" for demo purposes (Predefines close_gripper and placing strategy for placing the towel vertically).
  - **napkin**: "Fast button" for demo purposes (Predefines close_gripper and placing strategy for placing the napkin diagonally).
  - **diagonal_place**: This will place the object with a diagonal movement.
  - **vertical_place**: This will place the object with a vertical movement.
  - **rotating_place**: This will place the obejct first rotating the gripper 90º.
  - **dynamic_place**: -In progress-
- ***Configuration parameters***:
  - **handeye**: XYZ and RPY offsets for handeye transformation between camera and kinova base.
- ***Test pose parameters:***
  - **test**: Starts the state machine from initial state but for grasping the given position.
  - **frame_id**: Reference frame of the fiven position.
  - **grasp**: Grasping target pose for testing.

To execute the pick and place demo:
1. Adjust the handeye parameters according to camera's position wrt base robot.
2. To execute a predefined demo select towel or napkin and continue to the next step. Otherwise, introduce the closing percentage in close_gripper according to object's thickness and select the placing strategy (diagonal_place, vertical_place or rotating_place).
3. Place the folded object in the "pick" zone and press get_grasp_point.
4. Start the state machine pressing start_demo.

## Execution using ROSPlan

Compile the PDDL package that includes the action client (RPTutorial10.cpp):

``catkin_make --only-pkg-with-deps rosplan_planning_system``

After launching the previous launches, launch the the knowledge base, problem and planner interface to store the PDDL model and generate the problem and call the planner:

``cd PicknPlace/pnp_planner/launch``
``roslaunch rosplan_tutorial10.launch``

Generate the problem and the plan with the script:

`` ./tutorial04.bash``

<!-- To generate different plans, modify the init or goal conditions in the problem file "/pnp_planner/rosplan_problem.pddl". To check the output plan: 

`` `` -->

