(define (problem task)
(:domain picknplacepileclass)
(:objects
    towel hola - garment
    long short - grasp
    placevert placediag - placing
    grws rotws - workspace
    grasped placed notgrasped lifted - state
    home high_pose else drag_pose - position
    flat a b - defclass
)
(:init
    (garment_at towel rotws)

    (at_pose towel long)

    (garment_state towel notgrasped)

    (not (corners_pos_known towel))

    (robot_at else)

    (robot_empty)

    (not (known_obj hola))

    (defstate towel flat)


    (def_class a)

    (= (time_cost) 0)

    (= (place_qual) 0)

    (= (place_succ a placevert) 0)
    (= (place_succ b placevert) 10)
    (= (place_succ a placediag) 1)
    (= (place_succ b placediag) 1)

)
(:goal (and
    (garment_state towel placed)
))
(:metric minimize (+ (* 1 (time_cost)) (* 10 (place_qual))))
)
