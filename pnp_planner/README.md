
## Getting started

This repository includes the PDDL model (rosplan_domain.pddl and rosplan_problem.pddl), the launch files and the necessary ROSPlan packages.

- rosplan_domain/problem.pddl: Latest working domain
- rosplan_domain/problem_pileclass.pddl: Domain/problem with test modifications

## Execution PDDL domain

Having a domain and problem file, a plan can be generated using a solver, in our example we used the Fast-Forward solver [1] with cost minimization A* (s=3) and without heuristic search (w=0).

``export PATH=$PATH:~/Metric-FF-v2.1``

``ff -o domain_FOLDING.pddl -f problem_FOLDING.pddl -s 3 -w 0``

or 

``export PATH=$PATH:~/iri-lab/iri_ws/src/PicknPlace/ROSPlan/rosplan_planning_system/common/bin``

``Metric-FF -o rosplan_domain.pddl -f rosplan_problem.pddl -s 3 -w 0``

## PnP Demo execution with ROSPlan

The defined PDDL domain can be encapsulated into ROSPlan [2]. 

Launch the kinova, camera and demo:

``roslaunch pick_n_place camera_n_kinova.launch``

``roslaunch pick_n_place pick_n_place.launch``

Launch the knowledge base, problem and planner interface to store the PDDL model and generate the problem and call the planner:

``cd PicknPlace/pnp_planner/launch``

``roslaunch rosplan_tutorial10.launch``

or `rosplan_tutorial10_sim.launch` without launching the robot.

Press `plan_pddl_demo` in the reconfigure to generate problem, plan and parse. If the plan is correct (check with `rostopic echo /rosplan_planner_interface/planner_output -p -n 1`), dispatch the plan calling the service:

``rosservice call /rosplan_plan_dispatcher/dispatch_plan``

Every time that the demo cancels the plan (after check_corners and check_deformation), replan and disptach again.

<!-- Use Metric-FF to optimize cost function (instead of POPf) and rosplan_domain.pddl and rosplan_problem.pddl files which don't have durative-actions since Metric-FF does not support temporal planners. 

Generate the problem and the plan with the script:

`` ./tutorial04.bash``

Use pddl_simple_plan_parser and dispatcher to send actions one by one. -->


## Costs update

At this time, costs are introduced manually in the planner in rosplan_problem.pddl. Set the cloth-to-table and cloth-to-cloth costs as (= (place_succ towel A placevert) 17).
Costs are updated with the main_cost_update.py sccript in /pick_n_place/scripts. Check the related README file for obtaining the current and updated costs.


## Dependencies

- Fast-Forward solver [1]
    - https://fai.cs.uni-saarland.de/hoffmann/metric-ff.html (Download version 2.1)
- ROSPlan [2]
    - https://kcl-planning.github.io/ROSPlan/
    - https://planning.wiki/ref/pddl/requirements

