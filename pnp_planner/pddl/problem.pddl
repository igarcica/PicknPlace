(define (problem task)
(:domain picknplacepileclass)
(:objects
    table check waffle2 twlrag waffle1 - garment
    long short - grasp
    placevert placediag placerot - placing
    grws rotws - workspace
    grasped placed notgrasped lifted - state
    home high_pose else drag_pose - position
    flat a b c - defclass
)
(:init
    (garment_at twlrag grws)
    (garment_at check rotws)
    (garment_at twlrag rotws)
    (garment_at waffle1 rotws)
    (garment_at waffle2 rotws)

    (at_pose table long)
    (at_pose twlrag long)
    (at_pose waffle2 long)
    (at_pose check long)
    (at_pose waffle1 short)
    (at_pose waffle1 long)

    (garment_state table placed)
    (garment_state check placed)
    (garment_state twlrag placed)
    (garment_state waffle1 placed)
    (garment_state waffle2 lifted)

    (grasped_by check long)
    (grasped_by twlrag long)
    (grasped_by waffle1 long)
    (grasped_by waffle2 long)

    (corners_pos_known waffle2)
    (corners_pos_known check)
    (corners_pos_known twlrag)
    (corners_pos_known waffle1)

    (robot_at else)


    (known_obj check)
    (known_obj waffle2)
    (known_obj twlrag)
    (known_obj waffle1)

    (defstate check C)
    (defstate twlrag A)
    (defstate waffle1 A)
    (defstate waffle2 A)

    (obj_grasp_class check short B)
    (obj_grasp_class check long C)
    (obj_grasp_class twlrag short A)
    (obj_grasp_class twlrag long A)
    (obj_grasp_class waffle1 short A)
    (obj_grasp_class waffle1 long A)
    (obj_grasp_class waffle2 short A)
    (obj_grasp_class waffle2 long A)

    (on check table)
    (on twlrag check)
    (on waffle1 twlrag)


    (= (time_cost) 0)

    (= (place_qual) 0)

    (= (place_succ check a placevert) 2)
    (= (place_succ check a placediag) 2)
    (= (place_succ check a placerot) 2)
    (= (place_succ check b placevert) 6)
    (= (place_succ check b placediag) 5)
    (= (place_succ check b placerot) 5)
    (= (place_succ check c placevert) 3)
    (= (place_succ check c placediag) 1)
    (= (place_succ check c placerot) 1)
    (= (place_succ waffle2 a placevert) 11)
    (= (place_succ waffle2 a placediag) 9)
    (= (place_succ waffle2 a placerot) 8)
    (= (place_succ waffle2 b placevert) 27)
    (= (place_succ waffle2 b placediag) 5)
    (= (place_succ waffle2 b placerot) 2)
    (= (place_succ waffle2 c placevert) 26)
    (= (place_succ waffle2 c placediag) 10)
    (= (place_succ waffle2 c placerot) 11)
    (= (place_succ twlrag a placevert) 11)
    (= (place_succ twlrag a placediag) 9)
    (= (place_succ twlrag a placerot) 8)
    (= (place_succ twlrag b placevert) 27)
    (= (place_succ twlrag b placediag) 5)
    (= (place_succ twlrag b placerot) 2)
    (= (place_succ twlrag c placevert) 26)
    (= (place_succ twlrag c placediag) 10)
    (= (place_succ twlrag c placerot) 11)
    (= (place_succ waffle1 a placevert) 11)
    (= (place_succ waffle1 a placediag) 9)
    (= (place_succ waffle1 a placerot) 8)
    (= (place_succ waffle1 b placevert) 27)
    (= (place_succ waffle1 b placediag) 5)
    (= (place_succ waffle1 b placerot) 2)
    (= (place_succ waffle1 c placevert) 26)
    (= (place_succ waffle1 c placediag) 10)
    (= (place_succ waffle1 c placerot) 11)

)
(:goal (and
    (on check table)
    (on twlrag check)
    (on waffle1 twlrag)
    (on waffle2 waffle1)
    (at_pose twlrag long)
    (at_pose waffle1 long)
    (at_pose waffle2 long)
))
(:metric minimize (+ (* 1 (time_cost)) (* 2 (place_qual))))
)
