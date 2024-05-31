
## Getting started

This repository includes the PDDL model (rosplan_domain.pddl and rosplan_problem.pddl), the launch files and the necessary ROSPlan packages.

## Execution PDDL domain

Having a domain and problem file, a plan can be generated using a solver, in out example we used the Fast-Forward solver [1].

``export PATH=$PATH:~/Metric-FF-v2.1``
``ff -o domain_FOLDING.pddl -f problem_FOLDING.pddl -s 4 -w 1``

or 

``export PATH=$PATH:~/iri-lab/iri_ws/src/PicknPlace/ROSPlan/rosplan_planning_system/common/bin``
``Metric-FF -o rosplan_domain.pddl -f rosplan_problem.pddl -s 3 -w 1``

## Demo execution with ROSPlan

The defined PDDL domain can be encapsulated into ROSPlan [2]. 

Launch the kinova, camera and demo:

``roslaunch pick_n_place camera_n_kinova.launch``
``roslaunch pick_n_place pick_n_place.launch``

Launch the knowledge base, problem and planner interface to store the PDDL model and generate the problem and call the planner:

``cd PicknPlace/pnp_planner/launch``
``roslaunch rosplan_tutorial10.launch``

Generate the problem and the plan with the script:

`` ./tutorial04.bash``


## Dependencies

- Fast-Forward solver [1]
    - https://fai.cs.uni-saarland.de/hoffmann/metric-ff.html (Download version 2.1)
- ROSPlan [2]
    - https://kcl-planning.github.io/ROSPlan/
    - https://planning.wiki/ref/pddl/requirements

