
ff: parsing domain file
domain 'PICKNPLACEPILECLASS' defined
 ... done.
ff: parsing problem file
problem 'TASK' defined
 ... done.


translating negated cond for predicate CORNERS_POS_KNOWN
translating negated cond for predicate KNOWN_OBJ

ff: search configuration is weighted A* with weight 0.
Metric is ((1.00*[RF1](TIME_COST)1.00*[RF0](PLACE_QUAL)) - () + 0.00)
COST MINIMIZATION DONE (WITH cost-minimizing relaxed plans).

advancing to goal distance:    8
                               7
                               6
                               5
                               4
                               3
                               2
                               1
                               0

ff: found legal plan as follows
step    0: PLACEROT TOWEL TOWEL SHORT B
        1: HOME
        2: GO_HIGH
        3: NEW_OBJECT SHORT ROTWS
        4: CHECK_CORNERS TOWEL2 ROTWS SHORT
        5: HOME
        6: GRASP TOWEL2 SHORT B
        7: CHECK_DEFORMATION TOWEL2
        8: PLACEROT TOWEL2 TOWEL SHORT B
plan cost: 20.000000

time spent:    0.00 seconds instantiating 117 easy, 18 hard action templates
               0.00 seconds reachability analysis, yielding 33 facts and 43 actions
               0.00 seconds creating final representation with 27 relevant facts, 2 relevant fluents
               0.00 seconds computing LNF
               0.00 seconds building connectivity graph
               0.00 seconds searching, evaluating 1358 states, to a max depth of 0
               0.00 seconds total time

