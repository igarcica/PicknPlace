(define (problem PICKNPLACEpileclass)
(:domain PICKNPLACEpileclass)
(:objects
    towel hola - garment
	placevert placediag - placing
	multedges singledge - grasp
	grws rotws - workspace
	grasped placed notgrasped lifted - state
	home high_pose else drag_pose - position
	flat A B - defclass
)

(:init (garment_at towel rotws) (at_pose towel singledge) (garment_state towel notgrasped) (not (corners_pos_known towel)) (defstate towel flat)
		(not (known_obj hola))
		(robot_at else) (robot_empty)
		(= (time_cost) 0)
		(= (place_qual) 0)
		(= (place_succ A placevert) 0)
		(= (place_succ B placevert) 10)
		(= (place_succ A placediag) 1)
		(= (place_succ B placediag) 1)
		(obj_grasp_class towel multedges A) ;;towel grasped by multedges results in deformation class A
		(obj_grasp_class towel singledge B)
		(obj_grasp_class hola multedges A)
		(obj_grasp_class hola singledge B)
)

(:goal (and (garment_state hola placed)))

;;(:metric minimize (time_cost))
;;(:metric minimize (place_qual))
(:metric minimize (+ (* 1 (time_cost)) (* 10 (place_qual))))

)
