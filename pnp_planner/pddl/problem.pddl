(define (problem task)
(:domain picknplacepileclass)
(:objects
    table checkered1 checkered2 - garment
    long short - grasp
    placevert placediag placerot - placing
    grws rotws - workspace
    grasped placed notgrasped lifted - state
    home high_pose else drag_pose - position
    flat a b c - defclass
)
(:init
    (garment_at checkered1 rotws)
    (garment_at checkered2 rotws)

    (at_pose table short)
    (at_pose checkered2 short)
    (at_pose checkered1 long)
    (at_pose checkered1 short)

    (garment_state table placed)
    (garment_state checkered1 placed)
    (garment_state checkered2 lifted)

    (grasped_by checkered1 short)
    (grasped_by checkered2 short)

    (corners_pos_known checkered2)
    (corners_pos_known checkered1)

    (robot_at else)


    (known_obj checkered1)
    (known_obj checkered2)

    (defstate checkered1 B)
    (defstate checkered2 B)

    (obj_grasp_class checkered1 short B)
    (obj_grasp_class checkered1 long C)
    (obj_grasp_class checkered2 short B)
    (obj_grasp_class checkered2 long C)

    (on checkered1 table)


    (= (time_cost) 0)

    (= (place_qual) 0)

    (= (place_succ checkered1 a placevert) 0)
    (= (place_succ checkered1 a placediag) 0)
    (= (place_succ checkered1 a placerot) 0)
    (= (place_succ checkered1 b placevert) 0)
    (= (place_succ checkered1 b placediag) 30)
    (= (place_succ checkered1 b placerot) 30)
    (= (place_succ checkered1 c placevert) 30)
    (= (place_succ checkered1 c placediag) 30)
    (= (place_succ checkered1 c placerot) 30)
    (= (place_succ checkered2 a placevert) 0)
    (= (place_succ checkered2 a placediag) 0)
    (= (place_succ checkered2 a placerot) 0)
    (= (place_succ checkered2 b placevert) 0)
    (= (place_succ checkered2 b placediag) 30)
    (= (place_succ checkered2 b placerot) 30)
    (= (place_succ checkered2 c placevert) 30)
    (= (place_succ checkered2 c placediag) 30)
    (= (place_succ checkered2 c placerot) 30)

)
(:goal (and
    (on checkered1 table)
    (on checkered2 checkered1)
))
(:metric minimize (+ (time_cost) (* 3 (place_qual))))
)
