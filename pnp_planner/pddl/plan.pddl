
ff: parsing domain file
domain 'PICKNPLACEPILECLASS' defined
 ... done.
ff: parsing problem file
problem 'TASK' defined
 ... done.


translating negated cond for predicate CORNERS_POS_KNOWN
translating negated cond for predicate KNOWN_OBJ

ff: search configuration is weighted A* with weight 0.
Metric is ((1.00*[RF1](TIME_COST)10.00*[RF0](PLACE_QUAL)) - () + 0.00)
COST MINIMIZATION DONE (WITH cost-minimizing relaxed plans).

advancing to goal distance:    5
                               4
                               3
                               2
                               1
                               0

ff: found legal plan as follows
step    0: GO_HIGH
        1: CHECK_CORNERS TOWEL
        2: HOME
        3: GRASP TOWEL LONG A
        4: CHECK_DEFORMATION TOWEL
        5: PLACEVERT TOWEL TOWEL LONG A
plan cost: 6.000000

time spent:    0.00 seconds instantiating 63 easy, 18 hard action templates
               0.00 seconds reachability analysis, yielding 35 facts and 48 actions
               0.00 seconds creating final representation with 32 relevant facts, 2 relevant fluents
               0.00 seconds computing LNF
               0.00 seconds building connectivity graph
               0.00 seconds searching, evaluating 14 states, to a max depth of 0
               0.00 seconds total time

