
ff: parsing domain file
domain 'PICKNPLACETEST' defined
 ... done.
ff: parsing problem file
problem 'TASK' defined
 ... done.



ff: search configuration is weighted A* with weight 1.
Metric is ((1.00*[RF0](PLACE_QUAL)) - () + 0.00)
COST MINIMIZATION DONE (WITH cost-minimizing relaxed plans).

advancing to goal distance:    5
                               4
                               3
                               2
                               1
                               0

ff: found legal plan as follows
step    0: DRAG TOWEL
        1: ROTATE TOWEL SINGLEDGE MULTEDGES
        2: CHECK_CORNERS TOWEL
        3: HOME TOWEL
        4: GRASP TOWEL MULTEDGES
        5: CHECK_DEFORMATION TOWEL
        6: PLACEDIAG TOWEL MULTEDGES
plan cost: 5.000000

time spent:    0.00 seconds instantiating 14 easy, 0 hard action templates
               0.00 seconds reachability analysis, yielding 12 facts and 14 actions
               0.00 seconds creating final representation with 12 relevant facts, 2 relevant fluents
               0.00 seconds computing LNF
               0.00 seconds building connectivity graph
               0.00 seconds searching, evaluating 13 states, to a max depth of 0
               0.00 seconds total time

