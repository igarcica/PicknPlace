(define (problem task)
(:domain picknplacepileclass)
(:objects
    towel hola - garment
    long short - grasp
    placevert placediag placerot - placing
    grws rotws - workspace
    grasped placed notgrasped lifted - state
    home high_pose else drag_pose - position
    flat a b c - defclass
)
(:init
    (garment_at towel grws)
    (garment_at hola rotws)

    (at_pose towel long)
    (at_pose hola long)

    (garment_state towel placed)
    (garment_state hola lifted)

    (not (corners_pos_known towel))
    (corners_pos_known hola)

    (robot_at else)


    (known_obj hola)

    (defstate towel A)
    (defstate hola A)


    (def_class towel a)
    (def_class hola a)

    (= (time_cost) 0)

    (= (place_qual) 0)

    (= (place_succ a placevert) 0)
    (= (place_succ b placevert) 30)
    (= (place_succ c placevert) 60)
    (= (place_succ a placediag) 10)
    (= (place_succ b placediag) 15)
    (= (place_succ c placediag) 31)
    (= (place_succ a placerot) 11)
    (= (place_succ b placerot) 20)
    (= (place_succ c placerot) 5)

)
(:goal (and
    (on hola towel)
))
(:metric minimize (+ (* 5 (time_cost)) (* 10 (place_qual))))
)
