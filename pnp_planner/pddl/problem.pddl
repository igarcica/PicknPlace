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
    (garment_at waffle2 rotws)
    (garment_at waffle1 grws)

    (at_pose waffle2 long)
    (at_pose table long)
    (at_pose waffle1 long)

    (garment_state table placed)
    (garment_state waffle2 notgrasped)
    (garment_state waffle1 lifted)

    (grasped_by waffle1 long)

    (corners_pos_known waffle1)
    (not (corners_pos_known waffle2))

    (robot_at else)


    (known_obj waffle1)
    (not (known_obj waffle2))

    (defstate waffle2 flat)
    (defstate waffle1 A)

    (obj_grasp_class waffle1 short A)
    (obj_grasp_class waffle1 long A)
    (obj_grasp_class waffle2 short A)
    (obj_grasp_class waffle2 long A)



    (= (time_cost) 0)

    (= (place_qual) 0)

    (= (place_succ waffle1 a placevert) 1)
    (= (place_succ waffle1 a placediag) 1)
    (= (place_succ waffle1 a placerot) 0)
    (= (place_succ waffle1 b placevert) 5)
    (= (place_succ waffle1 b placediag) 4)
    (= (place_succ waffle1 b placerot) 4)
    (= (place_succ waffle1 c placevert) 1)
    (= (place_succ waffle1 c placediag) 1)
    (= (place_succ waffle1 c placerot) 1)
    (= (place_succ waffle2 a placevert) 2)
    (= (place_succ waffle2 a placediag) 2)
    (= (place_succ waffle2 a placerot) 0)
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
