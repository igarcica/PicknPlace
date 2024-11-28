(define (problem PICKNPLACEtest)
(:domain PICKNPLACEtest)
(:objects
    towel1 towel2 - garment
	placevert placediag - placing
	multedges singledge - grasp
	grws rotws - workspace
	grasped placed notgrasped lifted - state
	unknown known - corners
	home high_pose else - position
	flat A B C - defclass
)

(:init (garment_state towel1 notgrasped) (garment_at towel1 grws) (at_pose towel1 multedges) (corners_pos towel1 unknown) (defstate towel1 flat)
		(robot_at else)

)

(:goal (and (robot_at home)))

;;(:metric minimize (time_cost))
;;(:metric minimize (place_qual))
(:metric minimize (+ (* 1 (time_cost)) (* 100 (place_qual))))

)