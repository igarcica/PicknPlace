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
;;(:init (garment_state towel placed) (defstate towel2 flat) (at_pose towel long)
		(not (known_obj towel2))
		(robot_at else) (robot_empty)
		(= (time_cost) 0)
		(= (place_qual) 0)
		;;placing costs
		(= (place_succ towel A placevert) 100)
		(= (place_succ towel B placevert) 100)
		(= (place_succ towel C placevert) 15)
		(= (place_succ towel C placediag) 15)
		(= (place_succ towel C placerot) 9)
		;;piling costs
		(= (place_succ towel2 A placevert) 100)
		(= (place_succ towel2 B placevert) 100)
		(= (place_succ towel2 C placevert) 45)
		(= (place_succ towel2 C placediag) 30)
		(= (place_succ towel2 C placerot) 43)

		(obj_grasp_class long C) ;;towel grasped by multedges results in deformation class A
		(obj_grasp_class short B) ;; It can be the same as with long, the second object will adapt to the first object's init pose
)

;;(:goal (and (garment_state towel placed) ))
;;(:goal (and (on hola towel) (on teta hola) ))
(:goal (and (on towel2 towel) ))

;;(:metric minimize (time_cost))
(:metric minimize (place_qual))
;;(:metric minimize (+ (* 5 (time_cost)) (* 10 (place_qual))))

)