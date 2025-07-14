(define (problem PICKNPLACEpileclass)
(:domain PICKNPLACEpileclass)
(:objects
    placed_obj piled_obj - garment
	placevert placediag placerot - placing
	long short - grasp
	grws rotws - workspace
	grasped placed notgrasped lifted - state
	home high_pose else drag_pose - position
	flat A B C - defclass
)

(:init (garment_state placed_obj notgrasped) (not (corners_pos_known placed_obj)) (defstate placed_obj flat) ;;place object (cloth-to-table costs)
;;(:init (garment_state placed_obj placed) (defstate piled_obj flat) (at_pose placed_obj short) ;;pile object (cloth-to-cloth costs)
		(not (known_obj piled_obj))
;;		(known_obj piled_obj) (corners_pos_known piled_obj) (garment_state piled_obj notgrasped) (garment_at piled_obj grws) (at_pose piled_obj long) ;; to plan without robot
		(robot_at else) (robot_empty)
		(= (time_cost) 0)
		(= (place_qual) 0)

		;;placing costs
		(= (place_succ placed_obj A placevert) 17)
		(= (place_succ placed_obj A placediag) 1)
		(= (place_succ placed_obj A placerot) 1)
		(= (place_succ placed_obj B placevert) 8)
		(= (place_succ placed_obj B placediag) 22)
		(= (place_succ placed_obj B placerot) 5)
		(= (place_succ placed_obj C placevert) 30)
		(= (place_succ placed_obj C placediag) 25)
		(= (place_succ placed_obj C placerot) 6)
		;;piling costs
		(= (place_succ piled_obj A placevert) 7)
		(= (place_succ piled_obj A placediag) 2)
		(= (place_succ piled_obj A placerot) 4)
		(= (place_succ piled_obj B placevert) 30)
		(= (place_succ piled_obj B placediag) 14)
		(= (place_succ piled_obj B placerot) 8)
		(= (place_succ piled_obj C placevert) 44)
		(= (place_succ piled_obj C placediag) 30)
		(= (place_succ piled_obj C placerot) 30)

		(obj_grasp_class long B) ;;placed_obj grasped by multedges results in deformation class A
		(obj_grasp_class short A) ;; It can be the same as with long, the second object will adapt to the first object's init pose
)

;;(:goal (and (garment_state placed_obj placed) ))
(:goal (and (on piled_obj placed_obj) ))

;;(:metric minimize (time_cost))
;;(:metric minimize (place_qual))
(:metric minimize (+ (time_cost) (place_qual))) ;; to select placing actions based on simplicity (vertical, diagonal, rotating)
;;(:metric minimize (+ (* 5 (time_cost)) (* 10 (place_qual))))

)