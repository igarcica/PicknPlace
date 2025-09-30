;; piles of more than 2 objects with different deformation classes depending on the cloth
;; tarda mucho en sacar un plan pero lo saca (aprox 50 segs)

(define (problem PICKNPLACEpileclass)
(:domain PICKNPLACEpileclass)
(:objects
    table twlrag waffle1 waffle2 checkered1 checkered2 checkered3 linrag - garment
	placevert placediag placerot - placing
	long short - grasp
	grws rotws - workspace
	grasped placed notgrasped lifted - state
	home high_pose else drag_pose - position
	flat A B C - defclass
)

;;(:init (garment_state twlrag placed) (defstate waffle1 flat) (at_pose twlrag short) ;;pile object (cloth-to-cloth costs)

(:init (garment_state table placed) (robot_at else) (robot_empty)

		;; object to place (cloth-to-table costs)
		(known_obj twlrag) (garment_state twlrag notgrasped) (garment_at twlrag rotws) (at_pose twlrag short) (not (corners_pos_known twlrag)) (defstate twlrag flat) 
		;;(known_obj linrag) (known_obj checkered1) (garment_state checkered1 placed) (at_pose checkered1 short)

		;; objects to pile
		(not (known_obj waffle1)) (garment_state waffle1 notgrasped) (garment_at waffle1 rotws) (at_pose waffle1 long) (not (corners_pos_known waffle1)) (defstate waffle1 flat) 
		(not (known_obj waffle2)) (garment_state waffle2 notgrasped) (garment_at waffle2 rotws) (at_pose waffle2 long) (not (corners_pos_known waffle2)) (defstate waffle2 flat) 
		(not (known_obj checkered1)) (garment_state checkered1 notgrasped) (garment_at checkered1 rotws) (at_pose checkered1 short) (not (corners_pos_known checkered1)) (defstate checkered1 flat) 
		(not (known_obj checkered2)) (garment_state checkered2 notgrasped) (garment_at checkered2 rotws) (at_pose checkered2 short) (not (corners_pos_known checkered2)) (defstate checkered2 flat) 
		(not (known_obj checkered3)) (garment_state checkered3 notgrasped) (garment_at checkered3 rotws) (at_pose checkered3 long) (not (corners_pos_known checkered3)) (defstate checkered3 flat) 
		(not (known_obj linrag)) (garment_state linrag notgrasped) (garment_at linrag rotws) (at_pose linrag long) (not (corners_pos_known linrag)) (defstate linrag flat) 

		(= (time_cost) 0)
		(= (place_qual) 0)

		;;placing costs
		(= (place_succ twlrag A placevert) 2)
		(= (place_succ twlrag A placediag) 1)
		(= (place_succ twlrag A placerot) 3)
		(= (place_succ twlrag B placevert) 10)
		(= (place_succ twlrag B placediag) 1)
		(= (place_succ twlrag B placerot) 6)
		(= (place_succ twlrag C placevert) 3)
		(= (place_succ twlrag C placediag) 10)
		(= (place_succ twlrag C placerot) 1)

		;;piling costs
		(= (place_succ waffle1 A placevert) 7)
		(= (place_succ waffle1 A placediag) 1)
		(= (place_succ waffle1 A placerot) 4)
		(= (place_succ waffle1 B placevert) 3)
		(= (place_succ waffle1 B placediag) 1)
		(= (place_succ waffle1 B placerot) 2)
		(= (place_succ waffle1 C placevert) 10)
		(= (place_succ waffle1 C placediag) 10)
		(= (place_succ waffle1 C placerot) 1)

		(= (place_succ waffle2 A placevert) 7)
		(= (place_succ waffle2 A placediag) 1)
		(= (place_succ waffle2 A placerot) 4)
		(= (place_succ waffle2 B placevert) 3)
		(= (place_succ waffle2 B placediag) 14)
		(= (place_succ waffle2 B placerot) 9)
		(= (place_succ waffle2 C placevert) 1)
		(= (place_succ waffle2 C placediag) 30)
		(= (place_succ waffle2 C placerot) 30)

		(= (place_succ checkered1 A placevert) 7)
		(= (place_succ checkered1 A placediag) 3)
		(= (place_succ checkered1 A placerot) 4)
		(= (place_succ checkered1 B placevert) 5)
		(= (place_succ checkered1 B placediag) 1)
		(= (place_succ checkered1 B placerot) 3)
		(= (place_succ checkered1 C placevert) 10)
		(= (place_succ checkered1 C placediag) 10)
		(= (place_succ checkered1 C placerot) 10)

		(= (place_succ checkered2 A placevert) 7)
		(= (place_succ checkered2 A placediag) 3)
		(= (place_succ checkered2 A placerot) 4)
		(= (place_succ checkered2 B placevert) 5)
		(= (place_succ checkered2 B placediag) 1)
		(= (place_succ checkered2 B placerot) 3)
		(= (place_succ checkered2 C placevert) 10)
		(= (place_succ checkered2 C placediag) 10)
		(= (place_succ checkered2 C placerot) 10)

		(= (place_succ linrag A placevert) 7)
		(= (place_succ linrag A placediag) 3)
		(= (place_succ linrag A placerot) 4)
		(= (place_succ linrag B placevert) 10)
		(= (place_succ linrag B placediag) 1)
		(= (place_succ linrag B placerot) 3)
		(= (place_succ linrag C placevert) 10)
		(= (place_succ linrag C placediag) 10)
		(= (place_succ linrag C placerot) 10)


		(obj_grasp_class twlrag short A) ;; It can be the same as with long, the second object will adapt to the first object's init pose
		(obj_grasp_class twlrag long A)  ;; twlrag grasped by long edge results in deformation class A

		(obj_grasp_class waffle1 short A)
		(obj_grasp_class waffle1 long A) 

		(obj_grasp_class waffle2 short A)
		(obj_grasp_class waffle2 long A) 

		(obj_grasp_class checkered1 short B)
		(obj_grasp_class checkered1 long C) 

		(obj_grasp_class checkered2 short B)
		(obj_grasp_class checkered2 long C) 		

		(obj_grasp_class checkered3 short B)
		(obj_grasp_class checkered3 long C) 

		(obj_grasp_class linrag short B)
		(obj_grasp_class linrag long C) 
)

;;(:goal (and (garment_state twlrag placed) ))
(:goal (and (on twlrag table) (on waffle1 twlrag) (on checkered1 waffle1) (on linrag checkered1) 

))

;;(:metric minimize (time_cost))
;;(:metric minimize (place_qual))
(:metric minimize (+ (time_cost) (place_qual))) ;; to select placing actions based on simplicity (vertical, diagonal, rotating)
;;(:metric minimize (+ (* 5 (time_cost)) (* 10 (place_qual))))

)