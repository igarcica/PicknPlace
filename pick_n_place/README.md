Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
Author Irene Garcia-Camacho (igarcia@iri.upc.edu).

# Demo Pick and Place

This package is used to perform Piling of folded cloths. It can be used with cloths of different size, thickness and rigidity. Grasps larger edge of the cloth and based on its deformation after picked it places it using different placing trajectories. It can also be used to place in piles.

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

# Execution 

First launch the camera node and robot driver, in this example the rs camera and kinova robot:
Launch the camera and the kortex driver:

``roslaunch pick_n_place camera_n_kinova.launch``

Launch the nodes corresponding to the demo (iri_kinova_linear_movement, pick_n_place, vision_pick_place, prediction module and state estimation module):

``roslaunch pick_n_place picknplace_demo.launch``

This will also launch the RVIZ to visualize the perception system and rqt reconfigure to control the demo, which includes the following variables:

<!-- The rqt_reconfigure includes the following variables: -->

- ***Start SM:***
  - **drag**: Executes the drag action after sensing object's pose.
  - **rotate**: Executes the rotate action after sensing object's pose.
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

To execute the pick and pile demo (Naive approach w/o planning):
1. Adjust the handeye parameters according to camera's position wrt base robot.
2. To execute a predefined demo select towel or napkin and continue to the next step. Otherwise, introduce the closing percentage in close_gripper according to object's thickness and select the placing strategy (diagonal_place, vertical_place or rotating_place).
3. Place the folded object in the "pick" zone and press get_grasp_point.
4. Check if the selected grasp point is correct in RVIZ and start the state machine pressing start_demo.

This approach follows a finite state machine (FSM) pipeline to execute pick and place without taking into account the type of object and object properties, hence, not deciding the best grasp location and placing strategy. <!-- It picks the nearest edge and places it in the pile with a vertical placing motion.  -->



## Execution with planning using ROSPlan

The rqt_reconfigure parameters related to the planner are:
- ***Start SM:***
  - **plan_pddl_demo**: Generates pddl problem based on current state of KB.
- ***Object's properties***: 
  - **grasp_second_edge**: To force the grasp of the second nearest edge.
  - **n_objs_pile**: Number of objects to pile.
  - **object_name**: Object name (when the pile is of repeated objects).
  - **objs_to_pile**: List of objects to pile (comma-separated names).
  - **layers**: Labels for the number of layers of the object (4l, 6l, 8l, 12l or 16l). If the objects to pile are different (objs_to_pile is used) it should include the list of layers.s

0. Compile the PDDL package that includes the action client (RPTutorial10.cpp):

``catkin_make --only-pkg-with-deps rosplan_planning_system``

1. After launching the previous launches, launch the the knowledge base, problem and planner interface to store the PDDL model and generate the problem and call the planner:

``cd PicknPlace/pnp_planner/launch``
``roslaunch rosplan_tutorial10.launch``

<!--Generate the problem and the plan with the script:

`` ./tutorial04.bash`` -->

2. Set the object properties in the "C_Object_properties section of the reconfigure, including the number of objects to pile (n_objs_pile), the list of object names (objs_to_pile), the number of layers, and if it is desired to force to grasp the second nearest edge. 

3. Start the demo generating and parsing the plan activating the boolean ``plan_pddl_demo`` in the reconfigure and check if the generated plan is ok.

  <!-- 3.1. Check if the generated plan is ok: -->
 
<!-- ``rostopic echo /rosplan_planner_interface/planner_output -p -n 1`` -->

4. Dispatch plan:

``rosservice call /rosplan_plan_dispatcher/dispatch_plan``

This will start the demo by executing the sections of the SM according to the parsed actions by ROSPLAN. When it gets to "check_corners" action, you must activate the boolean `ok` in the reconfigure to select the detected grasp point.

When the replanning is active based on object's pose, predicted deformation class and sensed deformation class, each time it goes to check_corners and check_deformation it will preempt the current plan, so steps 3 and 4 will have to be repeated to generate the new plan and disptach it.

<!-- Note: Notice that the prediction module depends on object's properties, where number of layers, stiffness and friction cannot be infered through visual inspection. Therefore, these parameters are introduced through rqt_reconfigure. -->

# Saving data of execution

For saving the data, including RGB and PCD files of the zenithal image, check the README file from data/save_data/ folder.
<!-- The Piling system using planner makes use of a zenithal camera to predict and estimate the object's state and plan the actions to execute. In order to save this information, run:

- log_picknplace.txt automatically saves the planner information.
-  -->