(define (problem task)
(:domain picknplacepileclass)
(:objects
    table waffle1 waffle2 - garment
    long short - grasp
    placevert placediag placerot - placing
    grws rotws - workspace
    grasped placed notgrasped lifted - state
    home high_pose else drag_pose - position
    flat a b c - defclass
)
(:init
    (garment_at waffle1 rotws)
    (garment_at waffle2 rotws)

    (at_pose table long)
    (at_pose waffle1 long)
    (at_pose waffle2 long)

    (garment_state table placed)
    (garment_state waffle1 placed)
    (garment_state waffle2 lifted)

    (grasped_by waffle1 long)
    (grasped_by waffle2 long)

    (corners_pos_known waffle1)
    (corners_pos_known waffle2)

    (robot_at else)


    (known_obj waffle1)
    (known_obj waffle2)

    (defstate waffle1 A)
    (defstate waffle2 A)

    (obj_grasp_class waffle1 short A)
    (obj_grasp_class waffle1 long A)
    (obj_grasp_class waffle2 short A)
    (obj_grasp_class waffle2 long A)

    (on waffle1 table)


    (= (time_cost) 0)

    (= (place_qual) 0)

    (= (place_succ waffle1 a placevert) 3)
    (= (place_succ waffle1 a placediag) 1)
    (= (place_succ waffle1 a placerot) 1)
    (= (place_succ waffle1 b placevert) 5)
    (= (place_succ waffle1 b placediag) 4)
    (= (place_succ waffle1 b placerot) 4)
    (= (place_succ waffle1 c placevert) 1)
    (= (place_succ waffle1 c placediag) 1)
    (= (place_succ waffle1 c placerot) 1)
    (= (place_succ waffle2 a placevert) 9)
    (= (place_succ waffle2 a placediag) 3)
    (= (place_succ waffle2 a placerot) 2)
    (= (place_succ waffle2 b placevert) 30)
    (= (place_succ waffle2 b placediag) 4)
    (= (place_succ waffle2 b placerot) 4)
    (= (place_succ waffle2 c placevert) 27)
    (= (place_succ waffle2 c placediag) 10)
    (= (place_succ waffle2 c placerot) 11)

)
(:goal (and
    (on waffle1 table)
    (on waffle2 waffle1)
))
(:metric minimize (+ (* 1 (time_cost)) (* 2 (place_qual))))
)
