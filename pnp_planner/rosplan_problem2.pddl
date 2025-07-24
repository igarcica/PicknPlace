;; piles of more than 2 objects with different deformation classes depending on the cloth
;; tarda mucho en sacar un plan pero lo saca (aprox 50 segs)

(define (problem PICKNPLACEpileclass)
(:domain PICKNPLACEpileclass)
(:objects
    table towel1 towel2 towel3 towel4 - garment
	placevert placediag placerot - placing
	long short - grasp
	grws rotws - workspace
	grasped placed notgrasped lifted - state
	home high_pose else drag_pose - position
	flat A B C - defclass
)

;;(:init (garment_state towel1 placed) (defstate towel2 flat) (at_pose towel1 short) ;;pile object (cloth-to-cloth costs)

(:init (garment_state table placed) (robot_at else) (robot_empty)

		;; object to place (cloth-to-table costs)
		(known_obj towel1) (garment_state towel1 notgrasped) (garment_at towel1 rotws) (at_pose towel1 long) (not (corners_pos_known towel1)) (defstate towel1 flat) 
		;;(known_obj linrag) (known_obj towel4) (garment_state towel4 placed) (at_pose towel4 short)

		;; objects to pile
		(not (known_obj towel2)) (garment_state towel2 notgrasped) (garment_at towel2 rotws) (at_pose towel2 long) (not (corners_pos_known towel2)) (defstate towel2 flat) 
		(not (known_obj towel3)) (garment_state towel3 notgrasped) (garment_at towel3 rotws) (at_pose towel3 long) (not (corners_pos_known towel3)) (defstate towel3 flat) 
		(not (known_obj towel4)) (garment_state towel4 notgrasped) (garment_at towel4 rotws) (at_pose towel4 short) (not (corners_pos_known towel4)) (defstate towel4 flat) 

		(= (time_cost) 0)
		(= (place_qual) 0)

		;;placing costs
		(= (place_succ towel1 A placevert) 17)
		(= (place_succ towel1 A placediag) 1)
		(= (place_succ towel1 A placerot) 1)
		(= (place_succ towel1 B placevert) 8)
		(= (place_succ towel1 B placediag) 22)
		(= (place_succ towel1 B placerot) 6)
		(= (place_succ towel1 C placevert) 30)
		(= (place_succ towel1 C placediag) 25)
		(= (place_succ towel1 C placerot) 6)

		;;piling costs
		(= (place_succ towel2 A placevert) 7)
		(= (place_succ towel2 A placediag) 3)
		(= (place_succ towel2 A placerot) 4)
		(= (place_succ towel2 B placevert) 30)
		(= (place_succ towel2 B placediag) 14)
		(= (place_succ towel2 B placerot) 9)
		(= (place_succ towel2 C placevert) 30)
		(= (place_succ towel2 C placediag) 30)
		(= (place_succ towel2 C placerot) 30)

		(= (place_succ towel3 A placevert) 7)
		(= (place_succ towel3 A placediag) 3)
		(= (place_succ towel3 A placerot) 4)
		(= (place_succ towel3 B placevert) 30)
		(= (place_succ towel3 B placediag) 14)
		(= (place_succ towel3 B placerot) 9)
		(= (place_succ towel3 C placevert) 30)
		(= (place_succ towel3 C placediag) 30)
		(= (place_succ towel3 C placerot) 30)

		(= (place_succ towel4 A placevert) 7)
		(= (place_succ towel4 A placediag) 3)
		(= (place_succ towel4 A placerot) 4)
		(= (place_succ towel4 B placevert) 30)
		(= (place_succ towel4 B placediag) 14)
		(= (place_succ towel4 B placerot) 9)
		(= (place_succ towel4 C placevert) 30)
		(= (place_succ towel4 C placediag) 30)
		(= (place_succ towel4 C placerot) 30)



		(obj_grasp_class towel1 short A) ;; It can be the same as with long, the second object will adapt to the first object's init pose
		(obj_grasp_class towel1 long A)  ;; towel1 grasped by long edge results in deformation class A

		(obj_grasp_class towel2 short B)
		(obj_grasp_class towel2 long C) 

		(obj_grasp_class towel3 short B)
		(obj_grasp_class towel3 long C) 

		(obj_grasp_class towel4 short A)
		(obj_grasp_class towel4 long A) 
)

;;(:goal (and (garment_state towel1 placed) ))
(:goal (and (on towel1 table) (on towel2 towel1) (on towel3 towel2) (on towel4 towel3)

))

;;(:metric minimize (time_cost))
;;(:metric minimize (place_qual))
(:metric minimize (+ (time_cost) (place_qual))) ;; to select placing actions based on simplicity (vertical, diagonal, rotating)
;;(:metric minimize (+ (* 5 (time_cost)) (* 10 (place_qual))))

)