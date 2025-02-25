(define (problem PICKNPLACEpileclass)
(:domain PICKNPLACEpileclass)
(:objects
    towel towel2 - garment
	placevert placediag placerot - placing
	long short - grasp
	grws rotws - workspace
	grasped placed notgrasped lifted - state
	home high_pose else drag_pose - position
	flat A B C - defclass
)

(:init (garment_state towel notgrasped) (not (corners_pos_known towel)) (defstate towel flat)
		(not (known_obj towel2))
		(robot_at high_pose) (robot_empty)
		(= (time_cost) 0)
		(= (place_qual) 0)
		;;placing costs

		(= (place_succ towel A placerot) 0)
		(= (place_succ towel B placerot) 0)
		(= (place_succ towel C placerot) 0)
		;;piling costs
		(= (place_succ towel2 A placevert) 0)
		(= (place_succ towel2 B placevert) 0)
		(= (place_succ towel2 C placevert) 0)

		(obj_grasp_class long A) ;;towel grasped by multedges results in deformation class A
		(obj_grasp_class short B) ;; It can be the same as with long, the second object will adapt to the first object's init pose
		;;(def_class towel A)
)

;;(:goal (and (garment_state towel placed) ))
;;(:goal (and (on hola towel) (on teta hola) ))
(:goal (and (on towel2 towel) ))

;;(:metric minimize (time_cost))
(:metric minimize (place_qual))
;;(:metric minimize (+ (* 5 (time_cost)) (* 10 (place_qual))))

)