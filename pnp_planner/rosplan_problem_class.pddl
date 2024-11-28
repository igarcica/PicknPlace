(define (problem PICKNPLACEtest)
(:domain PICKNPLACEtest)
(:objects
    towel - garment
	placevert placediag - placing
	multedges singledge - grasp
	grws rotws - workspace
	grasped placed notgrasped lifted - state
	unknown known - corners
	home high_pose - position
	flat A B C - defclass
)

(:init (garment_obj towel) (garment_at grws) (at_pose multedges) (garment_state notgrasped) 
		(corners_pos unknown) (not (robot_at home)) (defstate flat)
		(= (time_cost) 0)
		(= (place_qual) 0)
		(= (place_succ A placevert) 10)
		(= (place_succ B placevert) 1)
		(= (place_succ A placediag) 20)
		(= (place_succ B placediag) 5)
		(obj_grasp_class towel multedges A) ;;towel grasped by multedges results in deformation class A
		(obj_grasp_class towel singledge B)
)

(:goal (and (garment_state placed)))

;;(:metric minimize (time_cost))
;;(:metric minimize (place_qual))
(:metric minimize (+ (* 1 (time_cost)) (* 100 (place_qual))))

)