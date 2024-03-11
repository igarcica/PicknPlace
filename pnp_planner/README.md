
## Execution PDDL domain

Having a domain and problem file, a plan can be generated using a solver, in out example we used the Fast-Forward solver [1].

``export PATH=$PATH:~/Metric-FF-v2.1``

``ff -o domain_FOLDING.pddl -f problem_FOLDING.pddl -s 4 -w 1``

## Execution ROSPlan

The defined PDDL domain can be encapsulated into ROSPlan [2]. 

Launch the knowledge base, problem and planner interface to store the PDDL model and generate the problem and call the planner:

``roslaunch rosplan_tutorial02.launch``

Generate the problem and the plan with the script:

`` ./tutorial.bash``


## Dependencies

- Fast-Forward solver [1]
- ROSPlan [2]

## References

[1] https://fai.cs.uni-saarland.de/hoffmann/metric-ff.html (Download version 2.1)
[2] https://kcl-planning.github.io/ROSPlan/
https://planning.wiki/ref/pddl/requirements

