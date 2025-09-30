
ff: parsing domain file
domain 'PICKNPLACEPILECLASS' defined
 ... done.
ff: parsing problem file
problem 'TASK' defined
 ... done.


translating negated cond for predicate CORNERS_POS_KNOWN

ff: search configuration is weighted A* with weight 0.
Metric is ((1.00*[RF1](TIME_COST)2.00*[RF0](PLACE_QUAL)) - () + 0.00)
COST MINIMIZATION DONE (WITH cost-minimizing relaxed plans).

advancing to goal distance:    4
                               3
                               2
                               1
                               0

ff: found legal plan as follows
step    0: HOME
        1: ROTATE PILLOWC LONG SHORT
        2: HOME
        3: GO_HIGH
        4: CHECK_CORNERS PILLOWC
        5: HOME
        6: GRASP PILLOWC SHORT B
        7: CHECK_DEFORMATION PILLOWC
        8: PLACEROT PILLOWC TOWEL TABLE SHORT B
plan cost: 8.000000

time spent:    0.00 seconds instantiating 662 easy, 26 hard action templates
               0.00 seconds reachability analysis, yielding 36 facts and 119 actions
               0.00 seconds creating final representation with 22 relevant facts, 2 relevant fluents
               0.00 seconds computing LNF
               0.00 seconds building connectivity graph
               0.00 seconds searching, evaluating 47 states, to a max depth of 0
               0.00 seconds total time

