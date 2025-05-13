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

;;(:init (garment_state towel notgrasped) (not (corners_pos_known towel)) (defstate towel flat)
(:init (garment_state towel placed) (defstate towel2 flat) (at_pose towel long)
		(not (known_obj towel2))
		(robot_at else) (robot_empty)
		(= (time_cost) 0)
		(= (place_qual) 0)
		;;placing costs
		(= (place_succ towel A placevert) 17)
		(= (place_succ towel A placediag) 1)
		(= (place_succ towel B placevert) 8)
		(= (place_succ towel B placediag) 22)
		(= (place_succ towel B placerot) 6)
		(= (place_succ towel C placevert) 30)
		(= (place_succ towel C placediag) 25)
		(= (place_succ towel C placediag) 6)

		;;piling costs
		(= (place_succ towel2 A placevert) 12)
		(= (place_succ towel2 A placediag) 6)
		(= (place_succ towel2 A placerot) 7)
		(= (place_succ towel2 B placevert) 30)
		(= (place_succ towel2 B placediag) 14)
		(= (place_succ towel2 B placerot) 9)
		(= (place_succ towel2 C placevert) 30)
		(= (place_succ towel2 C placediag) 30)
		(= (place_succ towel2 C placerot) 30)

		(obj_grasp_class long A) ;;towel grasped by multedges results in deformation class A
		(obj_grasp_class short A) ;; It can be the same as with long, the second object will adapt to the first object's init pose
)

;;(:goal (and (garment_state towel placed) ))
;;(:goal (and (on hola towel) (on teta hola) ))
(:goal (and (on towel2 towel) ))

;;(:metric minimize (time_cost))
(:metric minimize (place_qual))
;;(:metric minimize (+ (* 5 (time_cost)) (* 10 (place_qual))))

)