(define (problem task)
(:domain picknplacepileclass)
(:objects
    table towel pillowc - garment
    long short - grasp
    placevert placediag placerot - placing
    grws rotws - workspace
    grasped placed notgrasped lifted - state
    home high_pose else drag_pose - position
    flat a b c - defclass
)
(:init
    (garment_at towel rotws)
    (garment_at pillowc rotws)

    (at_pose table long)
    (at_pose towel long)
    (at_pose towel short)
    (at_pose pillowc long)

    (garment_state table placed)
    (garment_state pillowc notgrasped)
    (garment_state towel placed)

    (grasped_by towel long)

    (corners_pos_known pillowc)
    (corners_pos_known towel)

    (robot_at high_pose)

    (robot_empty)

    (known_obj towel)
    (known_obj pillowc)

    (defstate pillowc flat)
    (defstate towel A)

    (obj_grasp_class towel short A)
    (obj_grasp_class towel long A)
    (obj_grasp_class pillowc short B)
    (obj_grasp_class pillowc long C)

    (on towel table)


    (= (time_cost) 0)

    (= (place_qual) 0)

    (= (place_succ towel a placevert) 2)
    (= (place_succ towel a placediag) 2)
    (= (place_succ towel a placerot) 2)
    (= (place_succ towel b placevert) 5)
    (= (place_succ towel b placediag) 5)
    (= (place_succ towel b placerot) 6)
    (= (place_succ towel c placevert) 1)
    (= (place_succ towel c placediag) 1)
    (= (place_succ towel c placerot) 1)
    (= (place_succ pillowc a placevert) 9)
    (= (place_succ pillowc a placediag) 4)
    (= (place_succ pillowc a placerot) 3)
    (= (place_succ pillowc b placevert) 27)
    (= (place_succ pillowc b placediag) 5)
    (= (place_succ pillowc b placerot) 3)
    (= (place_succ pillowc c placevert) 26)
    (= (place_succ pillowc c placediag) 10)
    (= (place_succ pillowc c placerot) 11)

)
(:goal (and
    (on towel table)
    (on pillowc towel)
))
(:metric minimize (+ (* 1 (time_cost)) (* 2 (place_qual))))
)
