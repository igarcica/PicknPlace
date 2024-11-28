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
		(= (place_succ A placevert) 1)
		(= (place_succ B placevert) 10)
		(= (place_succ C placevert) 10)
		(= (place_succ A placediag) 1)
		(= (place_succ B placediag) 1)
		(= (place_succ C placediag) 1)
		(= (place_succ A placerot) 0)
		(= (place_succ B placerot) 1)
		(= (place_succ C placerot) 1)
		(obj_grasp_class towel long A) ;;towel grasped by long multiple edges results in deformation class A
		(obj_grasp_class towel short B) 
		(obj_grasp_class hola long B)
		(obj_grasp_class hola short A)
		;;0,10,1,1,A,B,A
)

(:goal (and (on hola towel) ))

;;(:metric minimize (time_cost))
;;(:metric minimize (place_qual))
(:metric minimize (+ (* 1 (time_cost)) (* 10 (place_qual))))

)
