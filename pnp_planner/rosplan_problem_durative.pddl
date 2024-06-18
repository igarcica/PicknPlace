(define (problem PICKNPLACEtest)
(:domain PICKNPLACEtest)
(:objects
    towel napkin waffle - garment
	placevert placediag placerot - placing
	multedges singledge - grasp
	grws rotws grrotws - workspace
	grasped placed notgrasped lifted - state
	unknown known - corners
	home postgrasp - position
)

(:init (garment_obj towel) (garment_at grws) (at_pose singledge) (garment_state notgrasped) (corners_pos unknown) (robot_at postgrasp)
		(= (time_cost) 0)
		(= (place_qual) 0)
		(= (place_succ towel singledge placevert) 10)
		(= (place_succ towel multedges placevert) 2000)
		(= (place_succ towel singledge placediag) 3)
		(= (place_succ towel multedges placediag) 1)
)

(:goal (and (garment_state placed)))

(:metric minimize (time_cost))
;;(:metric minimize (+ (* 1 (time_cost)) (* 10 (place_qual))))

)