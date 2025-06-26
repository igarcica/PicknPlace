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
    (garment_at towel2 rotws)

    (at_pose towel short)
    (at_pose towel2 long)

    (garment_state towel placed)
    (garment_state towel2 notgrasped)

    (corners_pos_known towel2)

    (robot_at high_pose)

    (robot_empty)

    (known_obj towel2)

    (defstate towel2 flat)



    (obj_grasp_class long A)
    (obj_grasp_class short A)

    (= (time_cost) 0)

    (= (place_qual) 0)

    (= (place_succ towel a placevert) 17)
    (= (place_succ towel a placediag) 10)
    (= (place_succ towel a placerot) 10)
    (= (place_succ towel b placevert) 8)
    (= (place_succ towel b placediag) 22)
    (= (place_succ towel b placerot) 6)
    (= (place_succ towel c placevert) 30)
    (= (place_succ towel c placediag) 25)
    (= (place_succ towel c placerot) 6)
    (= (place_succ towel2 a placevert) 7)
    (= (place_succ towel2 a placediag) 3)
    (= (place_succ towel2 a placerot) 4)
    (= (place_succ towel2 b placevert) 30)
    (= (place_succ towel2 b placediag) 14)
    (= (place_succ towel2 b placerot) 9)
    (= (place_succ towel2 c placevert) 30)
    (= (place_succ towel2 c placediag) 30)
    (= (place_succ towel2 c placerot) 30)

)
(:goal (and
    (on towel2 towel)
))
(:metric minimize (place_qual))
)
