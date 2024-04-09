Warnings encountered when parsing Domain/Problem File

Errors: 0, warnings: 10
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/rosplan_domain.pddl: line: 32: Warning: Undeclared symbol: notgrasped
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/rosplan_domain.pddl: line: 35: Warning: Undeclared symbol: grasped
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/rosplan_domain.pddl: line: 57: Warning: Undeclared symbol: grrotws
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/rosplan_domain.pddl: line: 73: Warning: Undeclared symbol: lifted
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/rosplan_domain.pddl: line: 85: Warning: Undeclared symbol: placed
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/pddl/problem.pddl: line: 7: Warning: Re-declaration of symbol in same scope: grrotws
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/pddl/problem.pddl: line: 8: Warning: Re-declaration of symbol in same scope: grasped
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/pddl/problem.pddl: line: 8: Warning: Re-declaration of symbol in same scope: placed
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/pddl/problem.pddl: line: 8: Warning: Re-declaration of symbol in same scope: notgrasped
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/pddl/problem.pddl: line: 8: Warning: Re-declaration of symbol in same scope: lifted
Number of literals: 9
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
All the ground actions in this problem are compression-safe
Initial heuristic = 2.000
b (1.000 | 1.000);;;; Solution Found
; States evaluated: 4
; Cost: 0.000
; Time 0.00
0.000: (drag towel grws grrotws)  [1.000]
1.001: (grasp towel grrotws singledge)  [60.000]
