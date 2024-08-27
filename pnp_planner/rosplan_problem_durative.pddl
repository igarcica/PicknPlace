(define (problem PICKNPLACEtest)
(:domain PICKNPLACEtest)
(:objects
    towel - garment
	placevert placediag - placing
	multedges singledge - grasp
	grws rotws - workspace
	grasped placed notgrasped lifted - state
	unknown known - corners
	home postgrasp - position
)

(:init (garment_obj towel) (garment_at grws) (at_pose singledge) (garment_state notgrasped) 
		(corners_pos unknown) (not (robot_at home))
		(= (time_cost) 0)
		(= (place_qual) 0)
		(= (place_succ towel singledge placevert) 1000)
		(= (place_succ towel multedges placevert) 3)
		(= (place_succ towel singledge placediag) 5000)
		(= (place_succ towel multedges placediag) 1)
)

(:goal (and (garment_state placed)))

;;(:metric minimize (time_cost))
;;(:metric minimize (place_qual))
(:metric minimize (+ (* 1 (time_cost)) (* 100 (place_qual))))

)