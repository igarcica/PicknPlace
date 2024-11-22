(define (problem PICKNPLACEtest)
(:domain PICKNPLACEtest)
(:objects
    towel hola - garment
	placevert placediag - placing
	multedges singledge - grasp
	grws rotws - workspace
	grasped placed notgrasped lifted - state
	home high_pose else drag_pose - position
)

(:init (garment_at towel rotws) (at_pose towel singledge) (garment_state towel notgrasped) (not (corners_pos_known towel)) (robot_at else) (robot_empty)
		(= (time_cost) 0)
		(= (place_qual) 0)
		(= (place_succ towel singledge placevert) 1)
		(= (place_succ towel multedges placevert) 0)
		(= (place_succ towel singledge placediag) 5000)
		(= (place_succ towel multedges placediag) 3)
)

(:goal (and (garment_state towel placed)))

;;(:metric minimize (time_cost))
;;(:metric minimize (place_qual))
(:metric minimize (+ (* 1 (time_cost)) (* 100 (place_qual))))

)
