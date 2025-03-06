
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

advancing to goal distance:    9
                               8
                               7
                               6
                               5
                               4
                               3
                               2
                               1
                               0

ff: found legal plan as follows
step    0: HOME
        1: GRASP TOWEL LONG C
        2: CHECK_DEFORMATION TOWEL
        3: PLACEROT TOWEL TOWEL LONG C
        4: HOME
        5: GO_HIGH
        6: NEW_OBJECT SHORT ROTWS
        7: CHECK_CORNERS TOWEL2 ROTWS LONG
        8: HOME
        9: GRASP TOWEL2 LONG C
       10: CHECK_DEFORMATION TOWEL2
       11: PLACEDIAG TOWEL2 TOWEL LONG C
plan cost: 40.000000

time spent:    0.00 seconds instantiating 117 easy, 18 hard action templates
               0.00 seconds reachability analysis, yielding 37 facts and 63 actions
               0.00 seconds creating final representation with 35 relevant facts, 2 relevant fluents
               0.00 seconds computing LNF
               0.00 seconds building connectivity graph
               0.06 seconds searching, evaluating 10976 states, to a max depth of 0
               0.06 seconds total time

