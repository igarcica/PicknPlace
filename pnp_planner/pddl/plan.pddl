Warnings encountered when parsing Domain/Problem File

Errors: 0, warnings: 18
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/rosplan_domain.pddl: line: 34: Warning: Undeclared symbol: notgrasped
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/rosplan_domain.pddl: line: 35: Warning: Undeclared symbol: unknown
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/rosplan_domain.pddl: line: 38: Warning: Undeclared symbol: known
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/rosplan_domain.pddl: line: 48: Warning: Undeclared symbol: postgrasp
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/rosplan_domain.pddl: line: 51: Warning: Undeclared symbol: home
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/rosplan_domain.pddl: line: 67: Warning: Undeclared symbol: grasped
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/rosplan_domain.pddl: line: 89: Warning: Undeclared symbol: grrotws
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/rosplan_domain.pddl: line: 105: Warning: Undeclared symbol: lifted
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/rosplan_domain.pddl: line: 117: Warning: Undeclared symbol: placed
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/pddl/problem.pddl: line: 7: Warning: Re-declaration of symbol in same scope: grrotws
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/pddl/problem.pddl: line: 8: Warning: Re-declaration of symbol in same scope: grasped
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/pddl/problem.pddl: line: 8: Warning: Re-declaration of symbol in same scope: placed
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/pddl/problem.pddl: line: 8: Warning: Re-declaration of symbol in same scope: notgrasped
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/pddl/problem.pddl: line: 8: Warning: Re-declaration of symbol in same scope: lifted
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/pddl/problem.pddl: line: 9: Warning: Re-declaration of symbol in same scope: unknown
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/pddl/problem.pddl: line: 9: Warning: Re-declaration of symbol in same scope: known
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/pddl/problem.pddl: line: 10: Warning: Re-declaration of symbol in same scope: home
/home/userlab/iri-lab/iri_ws/src/PicknPlace/pnp_planner/pddl/problem.pddl: line: 10: Warning: Re-declaration of symbol in same scope: postgrasp
Number of literals: 13
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%]
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
69% of the ground temporal actions in this problem are compression-safe
Initial heuristic = 9.000
b (7.000 | 1.000)b (5.000 | 2.001)
Resorting to best-first search
b (8.000 | 1.000)b (7.000 | 1.000)b (6.000 | 1.000)b (5.000 | 2.001)b (4.000 | 2.001)b (3.000 | 62.002)b (2.000 | 62.002)b (1.000 | 63.003);;;; Solution Found
; States evaluated: 39
; Cost: 0.000
; Time 0.00
0.000: (check_corners towel)  [1.000]
1.001: (home towel)  [1.000]
2.002: (grasp towel grws singledge)  [60.000]
62.003: (check_deformation towel)  [1.000]
63.004: (placevert towel)  [1.000]
