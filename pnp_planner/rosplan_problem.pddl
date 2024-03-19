(define (problem PICKNPLACEtest)
(:domain PICKNPLACEtest)
(:objects
    towel napkin waffle - garment
	placevert placediag placerot - placing
	multedges singledge - grasp
	grws rotws grrotws - workspace
	grasped placed notgrasped lifted - state
)

(:init (garment_obj towel) (garment_at grws) (at_pose singledge) (garment_state notgrasped)
		(= (time_cost) 0)
)

(:goal (and (garment_state grasped)))

(:metric minimize (time_cost))

)