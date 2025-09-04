(define (problem task)
(:domain picknplacepileclass)
(:objects
    table waffle1 checkered1 - garment
    long short - grasp
    placevert placediag placerot - placing
    grws rotws - workspace
    grasped placed notgrasped lifted - state
    home high_pose else drag_pose - position
    flat a b c - defclass
)
(:init
    (garment_at waffle1 rotws)
    (garment_at checkered1 rotws)

    (at_pose table long)
    (at_pose checkered1 short)
    (at_pose table short)
    (at_pose waffle1 long)
    (at_pose waffle1 short)

    (garment_state table placed)
    (garment_state waffle1 placed)
    (garment_state checkered1 lifted)

    (grasped_by waffle1 long)
    (grasped_by checkered1 short)

    (corners_pos_known checkered1)
    (corners_pos_known waffle1)

    (robot_at else)


    (known_obj waffle1)
    (known_obj checkered1)

    (defstate waffle1 B)
    (defstate checkered1 B)

    (obj_grasp_class waffle1 short A)
    (obj_grasp_class waffle1 long A)
    (obj_grasp_class checkered1 short B)
    (obj_grasp_class checkered1 long C)

    (on waffle1 table)


    (= (time_cost) 0)

    (= (place_qual) 0)

    (= (place_succ waffle1 a placevert) 2)
    (= (place_succ waffle1 a placediag) 1)
    (= (place_succ waffle1 a placerot) 1)
    (= (place_succ waffle1 b placevert) 5)
    (= (place_succ waffle1 b placediag) 4)
    (= (place_succ waffle1 b placerot) 4)
    (= (place_succ waffle1 c placevert) 1)
    (= (place_succ waffle1 c placediag) 1)
    (= (place_succ waffle1 c placerot) 1)
    (= (place_succ checkered1 a placevert) 3)
    (= (place_succ checkered1 a placediag) 2)
    (= (place_succ checkered1 a placerot) 2)
    (= (place_succ checkered1 b placevert) 30)
    (= (place_succ checkered1 b placediag) 5)
    (= (place_succ checkered1 b placerot) 4)
    (= (place_succ checkered1 c placevert) 27)
    (= (place_succ checkered1 c placediag) 10)
    (= (place_succ checkered1 c placerot) 11)

)
(:goal (and
    (on waffle1 table)
    (on checkered1 waffle1)
))
(:metric minimize (+ (* 1 (time_cost)) (* 10 (place_qual))))
)
