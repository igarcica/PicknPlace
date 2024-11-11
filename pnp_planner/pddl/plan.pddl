
ff: parsing domain file
domain 'PICKNPLACETEST' defined
 ... done.
ff: parsing problem file
problem 'TASK' defined
 ... done.



ff: search configuration is weighted A* with weight 0.
Metric is ((1.00*[RF1](TIME_COST)100.00*[RF0](PLACE_QUAL)) - () + 0.00)
COST MINIMIZATION DONE (WITH cost-minimizing relaxed plans).

advancing to goal distance:    6
                               5
                               4
                               3
                               2
                               1
                               0

ff: found legal plan as follows
step    0: HOME TOWEL
        1: GO_HIGH TOWEL
        2: CHECK_CORNERS TOWEL
        3: HOME TOWEL
        4: ROTATE TOWEL SINGLEDGE MULTEDGES
        5: HOME TOWEL
        6: GO_HIGH TOWEL
        7: CHECK_CORNERS TOWEL
        8: HOME TOWEL
        9: GRASP TOWEL GRWS MULTEDGES
       10: CHECK_DEFORMATION TOWEL
       11: PLACEDIAG TOWEL MULTEDGES
plan cost: 311.000000

time spent:    0.00 seconds instantiating 17 easy, 0 hard action templates
               0.00 seconds reachability analysis, yielding 11 facts and 16 actions
               0.00 seconds creating final representation with 10 relevant facts, 2 relevant fluents
               0.00 seconds computing LNF
               0.00 seconds building connectivity graph
               0.00 seconds searching, evaluating 44 states, to a max depth of 0
               0.00 seconds total time

