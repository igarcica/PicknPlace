
ff: parsing domain file
domain 'PICKNPLACETEST' defined
 ... done.
ff: parsing problem file
problem 'TASK' defined
 ... done.



ff: search configuration is A*epsilon with weight 1.
Metric is ((1.00*[RF0](TIME-COST)) - () + 0.00)
COST MINIMIZATION DONE (WITH cost-minimizing relaxed plans).

Advancing to goal distance:    5
                               4
                               3
                               2
                               1
                               0

ff: found legal plan as follows
step    0: APPROACH TOWEL MULTEDGES GRWS GRROTWS
        1: ROTATE TOWEL MULTEDGES SINGLEDGE ROTWS
        2: GRASP TOWEL GRROTWS SINGLEDGE
        3: LIFT MULTEDGES
        4: PLACEVERT TOWEL MULTEDGES
plan cost: 5.000000

time spent:    0.00 seconds instantiating 128 easy, 0 hard action templates
               0.00 seconds reachability analysis, yielding 9 facts and 56 actions
               0.00 seconds creating final representation with 8 relevant facts, 1 relevant fluents
               0.00 seconds computing LNF
               0.00 seconds building connectivity graph
               0.00 seconds searching, evaluating 29 states, to a max depth of 0
               0.00 seconds total time

