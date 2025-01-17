(define (problem PICKNPLACEpileclass)
(:domain PICKNPLACEpileclass)
(:objects
    towel hola - garment
	placevert placediag placerot - placing
	long short - grasp
	grws rotws - workspace
	grasped placed notgrasped lifted - state
	home high_pose else drag_pose - position
	flat A B C - defclass
)

(:init (garment_at towel grws) (at_pose towel long) (garment_state towel notgrasped) (not (corners_pos_known towel)) (defstate towel flat)
		(not (known_obj hola))
		(robot_at else) (robot_empty)
		(= (time_cost) 0)
		(= (place_qual) 0)
		(= (place_succ A placevert) 0)
		(= (place_succ B placevert) 30)
		(= (place_succ C placevert) 60)
		(= (place_succ A placediag) 10)
		(= (place_succ B placediag) 15)
		(= (place_succ C placediag) 31)
		(= (place_succ A placerot) 11)
		(= (place_succ B placerot) 20)
		(= (place_succ C placerot) 5)
		(def_class towel A)
)

;;(:goal (and (garment_state towel placed) ))
;;(:goal (and (on hola towel) (on teta hola) ))
(:goal (and (on hola towel) ))

;;(:metric minimize (time_cost))
;;(:metric minimize (place_qual))
(:metric minimize (+ (* 5 (time_cost)) (* 10 (place_qual))))

)