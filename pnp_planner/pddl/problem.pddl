(define (problem task)
(:domain picknplacepileclass)
(:objects
    towel towel2 - garment
    long short - grasp
    placevert placediag placerot - placing
    grws rotws - workspace
    grasped placed notgrasped lifted - state
    home high_pose else drag_pose - position
    flat a b c - defclass
)
(:init
    (garment_at towel grws)

    (at_pose towel long)

    (garment_state towel notgrasped)

    (corners_pos_known towel)

    (robot_at high_pose)

    (robot_empty)

    (not (known_obj towel2))

    (defstate towel flat)



    (obj_grasp_class long C)
    (obj_grasp_class short B)

    (= (time_cost) 0)

    (= (place_qual) 0)

    (= (place_succ towel a placevert) 100)
    (= (place_succ towel b placevert) 100)
    (= (place_succ towel c placevert) 15)
    (= (place_succ towel c placediag) 15)
    (= (place_succ towel c placerot) 9)
    (= (place_succ towel2 a placevert) 100)
    (= (place_succ towel2 b placevert) 100)
    (= (place_succ towel2 c placevert) 45)
    (= (place_succ towel2 c placediag) 30)
    (= (place_succ towel2 c placerot) 43)

)
(:goal (and
    (on towel2 towel)
))
(:metric minimize (place_qual))
)
