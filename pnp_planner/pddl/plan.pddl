
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

advancing to goal distance:   31
                              30
                              29
                              27
                              26
                              24
                              23
                              22
                              21
                              20
                              19
                              16
                              15
                              14
                              13
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
        2: CHECK_CORNERS TWLRAG
        3: HOME
        4: ROTATE TWLRAG LONG SHORT
        5: HOME
        6: GO_HIGH
        7: CHECK_CORNERS TWLRAG
        8: HOME
        9: GRASP TWLRAG SHORT A
       10: CHECK_DEFORMATION TWLRAG
       11: PLACEDIAG TWLRAG TABLE WAFFLE1 SHORT A
       12: HOME
       13: GO_HIGH
       14: CHECK_CORNERS WAFFLE1
       15: HOME
       16: ROTATE WAFFLE1 LONG SHORT
       17: HOME
       18: GO_HIGH
       19: CHECK_CORNERS WAFFLE1
       20: HOME
       21: GRASP WAFFLE1 SHORT B
       22: CHECK_DEFORMATION WAFFLE1
       23: PLACEROT WAFFLE1 TWLRAG CHECKERED1 SHORT B
       24: HOME
       25: GO_HIGH
       26: CHECK_CORNERS CHECKERED1
       27: HOME
       28: GRASP CHECKERED1 SHORT B
       29: CHECK_DEFORMATION CHECKERED1
       30: PLACEROT CHECKERED1 WAFFLE1 CHECKERED2 SHORT B
       31: HOME
       32: GO_HIGH
       33: CHECK_CORNERS CHECKERED2
       34: HOME
       35: GRASP CHECKERED2 SHORT B
       36: CHECK_DEFORMATION CHECKERED2
       37: PLACEROT CHECKERED2 CHECKERED1 LINRAG SHORT B
       38: HOME
       39: GO_HIGH
       40: CHECK_CORNERS LINRAG
       41: HOME
       42: ROTATE LINRAG LONG SHORT
       43: HOME
       44: GO_HIGH
       45: CHECK_CORNERS LINRAG
       46: HOME
       47: GRASP LINRAG SHORT B
       48: CHECK_DEFORMATION LINRAG
       49: PLACEROT LINRAG CHECKERED2 TABLE SHORT B
plan cost: 59.000000

time spent:    0.00 seconds instantiating 12327 easy, 66 hard action templates
               0.00 seconds reachability analysis, yielding 169 facts and 4283 actions
               0.00 seconds creating final representation with 159 relevant facts, 2 relevant fluents
               0.00 seconds computing LNF
               0.02 seconds building connectivity graph
               1.81 seconds searching, evaluating 10421 states, to a max depth of 0
               1.83 seconds total time

