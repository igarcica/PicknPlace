
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

advancing to goal distance:    7
                               6
                               5
                               4
                               3
                               2
                               1
                               0

ff: found legal plan as follows
step    0: PLACEROT WAFFLE1 TABLE WAFFLE2 LONG A
        1: HOME
        2: GO_HIGH
        3: CHECK_CORNERS WAFFLE2
        4: HOME
        5: GRASP WAFFLE2 LONG A
        6: CHECK_DEFORMATION WAFFLE2
        7: PLACEROT WAFFLE2 WAFFLE1 TABLE LONG A
plan cost: 8.000000

time spent:    0.00 seconds instantiating 662 easy, 26 hard action templates
               0.00 seconds reachability analysis, yielding 37 facts and 83 actions
               0.00 seconds creating final representation with 27 relevant facts, 2 relevant fluents
               0.00 seconds computing LNF
               0.00 seconds building connectivity graph
               0.00 seconds searching, evaluating 60 states, to a max depth of 0
               0.00 seconds total time

