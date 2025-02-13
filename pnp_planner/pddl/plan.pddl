
ff: parsing domain file
domain 'PICKNPLACEPILECLASS' defined
 ... done.
ff: parsing problem file
problem 'TASK' defined
 ... done.


translating negated cond for predicate CORNERS_POS_KNOWN
translating negated cond for predicate KNOWN_OBJ

ff: search configuration is weighted A* with weight 0.
Metric is ((1.00*[RF0](PLACE_QUAL)) - () + 0.00)
COST MINIMIZATION DONE (WITH cost-minimizing relaxed plans).

advancing to goal distance:    5
                               4
                               2
                               1
                               0

ff: found legal plan as follows
step    0: HOME
        1: GRASP TOWEL LONG A
        2: CHECK_DEFORMATION TOWEL
        3: PLACEVERT TOWEL TOWEL LONG A
plan cost: 0.000000

time spent:    0.00 seconds instantiating 117 easy, 18 hard action templates
               0.00 seconds reachability analysis, yielding 37 facts and 79 actions
               0.00 seconds creating final representation with 35 relevant facts, 2 relevant fluents
               0.00 seconds computing LNF
               0.00 seconds building connectivity graph
               0.00 seconds searching, evaluating 13 states, to a max depth of 0
               0.00 seconds total time

