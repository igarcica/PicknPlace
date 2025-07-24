
ff: parsing domain file
domain 'PICKNPLACEPILECLASS' defined
 ... done.
ff: parsing problem file
problem 'TASK' defined
 ... done.


translating negated cond for predicate CORNERS_POS_KNOWN

ff: search configuration is weighted A* with weight 0.
Metric is ((1.00*[RF1](TIME_COST)1.00*[RF0](PLACE_QUAL)) - () + 0.00)
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
step    0: PLACEROT CHECKERED2 CHECKERED1 LINRAG LONG B
        1: HOME
        2: GO_HIGH
        3: CHECK_CORNERS LINRAG
        4: HOME
        5: GRASP LINRAG LONG A
        6: CHECK_DEFORMATION LINRAG
        7: PLACEDIAG LINRAG CHECKERED2 TABLE LONG A
plan cost: 18.000000

time spent:    0.00 seconds instantiating 12327 easy, 66 hard action templates
               0.00 seconds reachability analysis, yielding 116 facts and 843 actions
               0.00 seconds creating final representation with 79 relevant facts, 2 relevant fluents
               0.00 seconds computing LNF
               0.00 seconds building connectivity graph
               0.01 seconds searching, evaluating 418 states, to a max depth of 0
               0.01 seconds total time

