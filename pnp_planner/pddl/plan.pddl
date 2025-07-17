
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

advancing to goal distance:   18
                              17
                              16
                              15
                              14
                              13
                              12
                              11
                              10
                               9
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
        1: GO_HIGH
        2: CHECK_CORNERS TOWEL
        3: HOME
        4: DRAG TOWEL
        5: GO_HIGH
        6: CHECK_CORNERS TOWEL
        7: HOME
        8: ROTATE TOWEL SHORT LONG
        9: HOME
       10: GO_HIGH
       11: CHECK_CORNERS TOWEL
       12: HOME
       13: GRASP TOWEL LONG B
       14: CHECK_DEFORMATION TOWEL
       15: PLACEVERT TOWEL TABLE WAFFLE1 LONG B
       16: HOME
       17: GO_HIGH
       18: CHECK_CORNERS WAFFLE1
       19: HOME
       20: GRASP WAFFLE1 LONG A
       21: CHECK_DEFORMATION WAFFLE1
       22: PLACEVERT WAFFLE1 TOWEL WAFFLE2 LONG A
       23: HOME
       24: GO_HIGH
       25: CHECK_CORNERS WAFFLE2
       26: HOME
       27: GRASP WAFFLE2 LONG C
       28: CHECK_DEFORMATION WAFFLE2
       29: PLACEVERT WAFFLE2 WAFFLE1 TABLE LONG C
plan cost: 11.000000

time spent:    0.00 seconds instantiating 12327 easy, 66 hard action templates
               0.00 seconds reachability analysis, yielding 170 facts and 5052 actions
               0.00 seconds creating final representation with 161 relevant facts, 2 relevant fluents
               0.00 seconds computing LNF
               0.02 seconds building connectivity graph
               3.78 seconds searching, evaluating 12138 states, to a max depth of 0
               3.80 seconds total time

