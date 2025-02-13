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
    (garment_at towel rotws)

    (at_pose towel long)

    (garment_state towel notgrasped)

    (corners_pos_known towel)

    (robot_at high_pose)

    (robot_empty)

    (not (known_obj towel2))

    (defstate towel flat)



    (obj_grasp_class long A)
    (obj_grasp_class short B)

    (= (time_cost) 0)

    (= (place_qual) 0)

    (= (place_succ towel a placevert) 0)
    (= (place_succ towel b placevert) 10)
    (= (place_succ towel c placevert) 60)
    (= (place_succ towel a placediag) 10)
    (= (place_succ towel b placediag) 1)
    (= (place_succ towel c placediag) 31)
    (= (place_succ towel a placerot) 11)
    (= (place_succ towel b placerot) 20)
    (= (place_succ towel c placerot) 5)
    (= (place_succ towel2 a placevert) 100)
    (= (place_succ towel2 b placevert) 10)
    (= (place_succ towel2 c placevert) 60)
    (= (place_succ towel2 a placediag) 100)
    (= (place_succ towel2 b placediag) 15)
    (= (place_succ towel2 c placediag) 31)
    (= (place_succ towel2 a placerot) 110)
    (= (place_succ towel2 b placerot) 0)
    (= (place_succ towel2 c placerot) 5)

)
(:goal (and
    (garment_state towel placed)
))
(:metric minimize (place_qual))
)
