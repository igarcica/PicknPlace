(define (problem PICKNPLACEpileclass)
  (:domain PICKNPLACEpileclass)

  (:objects
    towel waffle1 waffle2 checkered1 checkered2 - garment
    placevert placediag placerot - placing
    long short - grasp
    grws rotws - workspace
    grasped placed notgrasped lifted - state
    home high_pose else drag_pose - position
    flat A B C - defclass
  )

  (:init
    ;; Towel 1 - base towel
    (known_obj towel)
    (garment_state towel notgrasped)
    (not (corners_pos_known towel))
    (defstate towel flat)

    ;; Other towels to be piled
    (not (known_obj waffle1))

    ;; Robot and world state
    (robot_at else)
    (robot_empty)

    ;; Metrics
    (= (time_cost) 0)
    (= (place_qual) 0)

    ;; Placing costs for towel (base)
    (= (place_succ towel A placevert) 17)
    (= (place_succ towel A placediag) 1)
    (= (place_succ towel A placerot) 1)
    (= (place_succ towel B placevert) 8)
    (= (place_succ towel B placediag) 22)
    (= (place_succ towel B placerot) 5)
    (= (place_succ towel C placevert) 30)
    (= (place_succ towel C placediag) 25)
    (= (place_succ towel C placerot) 6)

    ;; Placing costs for waffle1
    (= (place_succ waffle1 A placevert) 7)
    (= (place_succ waffle1 A placediag) 2)
    (= (place_succ waffle1 A placerot) 4)
    (= (place_succ waffle1 B placevert) 30)
    (= (place_succ waffle1 B placediag) 14)
    (= (place_succ waffle1 B placerot) 8)
    (= (place_succ waffle1 C placevert) 44)
    (= (place_succ waffle1 C placediag) 30)
    (= (place_succ waffle1 C placerot) 30)

    ;; Copy cost values for waffle2, checkered1, checkered2
    (= (place_succ waffle2 A placevert) 7)
    (= (place_succ waffle2 A placediag) 2)
    (= (place_succ waffle2 A placerot) 4)
    (= (place_succ waffle2 B placevert) 30)
    (= (place_succ waffle2 B placediag) 14)
    (= (place_succ waffle2 B placerot) 8)
    (= (place_succ waffle2 C placevert) 44)
    (= (place_succ waffle2 C placediag) 30)
    (= (place_succ waffle2 C placerot) 30)

    (= (place_succ checkered1 A placevert) 7)
    (= (place_succ checkered1 A placediag) 2)
    (= (place_succ checkered1 A placerot) 4)
    (= (place_succ checkered1 B placevert) 30)
    (= (place_succ checkered1 B placediag) 14)
    (= (place_succ checkered1 B placerot) 8)
    (= (place_succ checkered1 C placevert) 44)
    (= (place_succ checkered1 C placediag) 30)
    (= (place_succ checkered1 C placerot) 30)

    (= (place_succ checkered2 A placevert) 7)
    (= (place_succ checkered2 A placediag) 2)
    (= (place_succ checkered2 A placerot) 4)
    (= (place_succ checkered2 B placevert) 30)
    (= (place_succ checkered2 B placediag) 14)
    (= (place_succ checkered2 B placerot) 8)
    (= (place_succ checkered2 C placevert) 44)
    (= (place_succ checkered2 C placediag) 30)
    (= (place_succ checkered2 C placerot) 30)

    ;; Grasp classes
    (obj_grasp_class long A)
    (obj_grasp_class short B)
  )

  (:goal
    (and
      (on waffle1 towel)
      (on waffle2 waffle1)
    )
  )

  (:metric minimize (place_qual))
)