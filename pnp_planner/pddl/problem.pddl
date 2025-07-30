(define (problem task)
(:domain picknplacepileclass)
(:objects
    table twlrag waffle1 checkered1 - garment
    long short - grasp
    placevert placediag placerot - placing
    grws rotws - workspace
    grasped placed notgrasped lifted - state
    home high_pose else drag_pose - position
    flat a b c - defclass
)
(:init
    (garment_at waffle1 grws)
    (garment_at checkered1 grws)
    (garment_at twlrag rotws)

    (at_pose twlrag short)
    (at_pose table short)
    (at_pose waffle1 short)
    (at_pose checkered1 short)

    (garment_state table placed)
    (garment_state twlrag placed)
    (garment_state waffle1 placed)
    (garment_state checkered1 lifted)

    (grasped_by twlrag short)
    (grasped_by waffle1 short)
    (grasped_by checkered1 short)

    (corners_pos_known twlrag)
    (corners_pos_known waffle1)
    (corners_pos_known checkered1)

    (robot_at else)


    (known_obj twlrag)
    (known_obj waffle1)
    (known_obj checkered1)

    (defstate waffle1 A)
    (defstate twlrag A)
    (defstate checkered1 B)

    (obj_grasp_class waffle1 short B)
    (obj_grasp_class checkered1 short B)
    (obj_grasp_class twlrag short A)

    (on twlrag table)
    (on waffle1 twlrag)


    (= (time_cost) 0)

    (= (place_qual) 0)

    (= (place_succ twlrag a placevert) 17)
    (= (place_succ twlrag a placediag) 1)
    (= (place_succ twlrag a placerot) 1)
    (= (place_succ twlrag b placevert) 8)
    (= (place_succ twlrag b placediag) 22)
    (= (place_succ twlrag b placerot) 6)
    (= (place_succ twlrag c placevert) 30)
    (= (place_succ twlrag c placediag) 25)
    (= (place_succ twlrag c placerot) 6)
    (= (place_succ waffle1 a placevert) 7)
    (= (place_succ waffle1 a placediag) 3)
    (= (place_succ waffle1 a placerot) 4)
    (= (place_succ waffle1 b placevert) 30)
    (= (place_succ waffle1 b placediag) 14)
    (= (place_succ waffle1 b placerot) 9)
    (= (place_succ waffle1 c placevert) 30)
    (= (place_succ waffle1 c placediag) 30)
    (= (place_succ waffle1 c placerot) 30)
    (= (place_succ checkered1 a placevert) 7)
    (= (place_succ checkered1 a placediag) 7)
    (= (place_succ checkered1 a placerot) 4)
    (= (place_succ checkered1 b placevert) 30)
    (= (place_succ checkered1 b placediag) 14)
    (= (place_succ checkered1 b placerot) 9)
    (= (place_succ checkered1 c placevert) 30)
    (= (place_succ checkered1 c placediag) 30)
    (= (place_succ checkered1 c placerot) 30)

)
(:goal (and
    (on twlrag table)
    (on waffle1 twlrag)
    (on checkered1 waffle1)
))
(:metric minimize (+ (time_cost) (place_qual)))
)
