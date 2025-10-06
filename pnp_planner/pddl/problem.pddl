(define (problem task)
(:domain picknplacepileclass)
(:objects
    table cotnap1 cotnap2 - garment
    long short - grasp
    placevert placediag placerot - placing
    grws rotws - workspace
    grasped placed notgrasped lifted - state
    home high_pose else drag_pose - position
    flat a b c - defclass
)
(:init
    (garment_at cotnap1 rotws)
    (garment_at cotnap2 rotws)

    (at_pose cotnap1 long)
    (at_pose table long)
    (at_pose cotnap2 long)

    (garment_state table placed)
    (garment_state cotnap1 placed)
    (garment_state cotnap2 lifted)

    (grasped_by cotnap1 long)
    (grasped_by cotnap2 long)

    (corners_pos_known cotnap1)
    (corners_pos_known cotnap2)

    (robot_at else)


    (known_obj cotnap1)
    (known_obj cotnap2)

    (defstate cotnap1 C)
    (defstate cotnap2 C)

    (obj_grasp_class cotnap1 short B)
    (obj_grasp_class cotnap1 long C)
    (obj_grasp_class cotnap2 short B)
    (obj_grasp_class cotnap2 long C)

    (on cotnap1 table)


    (= (time_cost) 0)

    (= (place_qual) 0)

    (= (place_succ cotnap1 a placevert) 2)
    (= (place_succ cotnap1 a placediag) 2)
    (= (place_succ cotnap1 a placerot) 2)
    (= (place_succ cotnap1 b placevert) 5)
    (= (place_succ cotnap1 b placediag) 4)
    (= (place_succ cotnap1 b placerot) 4)
    (= (place_succ cotnap1 c placevert) 13)
    (= (place_succ cotnap1 c placediag) 1)
    (= (place_succ cotnap1 c placerot) 1)
    (= (place_succ cotnap2 a placevert) 9)
    (= (place_succ cotnap2 a placediag) 4)
    (= (place_succ cotnap2 a placerot) 3)
    (= (place_succ cotnap2 b placevert) 27)
    (= (place_succ cotnap2 b placediag) 4)
    (= (place_succ cotnap2 b placerot) 4)
    (= (place_succ cotnap2 c placevert) 26)
    (= (place_succ cotnap2 c placediag) 18)
    (= (place_succ cotnap2 c placerot) 29)

)
(:goal (and
    (on cotnap1 table)
    (at_pose cotnap1 long)
    (on cotnap2 cotnap1)
))
(:metric minimize (+ (* 1 (time_cost)) (* 2 (place_qual))))
)
