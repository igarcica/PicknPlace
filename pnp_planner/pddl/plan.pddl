
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
                              28
                              27
                              26
                              25
                              24
                              23
                              22
                              21
                              20
                              19
                              18
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
        4: GRASP TOWEL LONG A
        5: CHECK_DEFORMATION TOWEL
        6: PLACEVERT TOWEL TABLE CHECKERED1 LONG A
        7: HOME
        8: GO_HIGH
        9: CHECK_CORNERS CHECKERED1
       10: HOME
       11: ROTATE CHECKERED1 SHORT LONG
       12: HOME
       13: GO_HIGH
       14: CHECK_CORNERS CHECKERED1
       15: HOME
       16: GRASP CHECKERED1 LONG C
       17: CHECK_DEFORMATION CHECKERED1
       18: PLACEVERT CHECKERED1 TOWEL CHECKERED2 LONG C
       19: HOME
       20: GO_HIGH
       21: CHECK_CORNERS CHECKERED2
       22: HOME
       23: ROTATE CHECKERED2 SHORT LONG
       24: HOME
       25: GO_HIGH
       26: CHECK_CORNERS CHECKERED2
       27: HOME
       28: GRASP CHECKERED2 LONG C
       29: CHECK_DEFORMATION CHECKERED2
       30: PLACEVERT CHECKERED2 CHECKERED1 WAFFLE1 LONG C
       31: HOME
       32: GO_HIGH
       33: CHECK_CORNERS WAFFLE1
       34: HOME
       35: GRASP WAFFLE1 LONG C
       36: CHECK_DEFORMATION WAFFLE1
       37: PLACEVERT WAFFLE1 CHECKERED2 WAFFLE2 LONG C
       38: HOME
       39: GO_HIGH
       40: CHECK_CORNERS WAFFLE2
       41: HOME
       42: GRASP WAFFLE2 LONG C
       43: CHECK_DEFORMATION WAFFLE2
       44: PLACEVERT WAFFLE2 WAFFLE1 TABLE LONG C
plan cost: 17.000000

time spent:    0.00 seconds instantiating 12327 easy, 66 hard action templates
               0.00 seconds reachability analysis, yielding 169 facts and 5051 actions
               0.00 seconds creating final representation with 159 relevant facts, 2 relevant fluents
               0.01 seconds computing LNF
               0.02 seconds building connectivity graph
              47.50 seconds searching, evaluating 148515 states, to a max depth of 0
              47.53 seconds total time

