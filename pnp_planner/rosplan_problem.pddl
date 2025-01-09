(define (problem PICKNPLACEpileclass)
(:domain PICKNPLACEpileclass)
(:objects
    towel hola - garment
	placevert placediag - placing
	long short - grasp
	grws rotws - workspace
	grasped placed notgrasped lifted - state
	home high_pose else drag_pose - position
	flat A B - defclass
)

(:init (garment_at towel rotws) (at_pose towel long) (garment_state towel notgrasped) (not (corners_pos_known towel)) (defstate towel flat)
		(not (known_obj hola))
		(robot_at else) (robot_empty)
		(= (time_cost) 0)
		(= (place_qual) 0)
		(= (place_succ A placevert) 0)
		(= (place_succ B placevert) 10)
		(= (place_succ A placediag) 1)
		(= (place_succ B placediag) 1)
		;;(obj_grasp_class towel long A) ;;towel grasped by long multiple edges results in deformation class A
		;;(obj_grasp_class towel short B) 
		;;(obj_grasp_class hola long A)
		;;(obj_grasp_class hola short A)
		(def_class A)
)

(:goal (and (garment_state towel placed) ))
;;(:goal (and (on hola towel) ))
;;(:goal (and (robot_at home)))

;;(:metric minimize (time_cost))
;;(:metric minimize (place_qual))
(:metric minimize (+ (* 1 (time_cost)) (* 10 (place_qual))))

)
