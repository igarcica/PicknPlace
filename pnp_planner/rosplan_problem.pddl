(define (problem PICKNPLACEpileclass) 
(:domain PICKNPLACEpileclass)
(:objects 
 	 table cotnap1 cotnap2 - garment 
	 placevert placediag placerot - placing 
	 long short - grasp 
	 grws rotws - workspace 
	 grasped placed notgrasped lifted - state 
	 home high_pose else drag_pose - position 
	 flat A B C - defclass 
 ) 

(:init (garment_state table placed) (robot_at else) (robot_empty) 

	 ;; object to place (cloth-to-table costs) 
	 (known_obj cotnap1) (garment_state cotnap1 notgrasped) (garment_at cotnap1 grws) (at_pose cotnap1 short) (not (corners_pos_known cotnap1)) (defstate cotnap1 flat) 

	 ;; objects to pile (cloth-to-cloth costs) 
	 (not (known_obj cotnap2)) (garment_state cotnap2 notgrasped) (garment_at cotnap2 rotws) (at_pose cotnap2 short) (not (corners_pos_known cotnap2)) (defstate cotnap2 flat) 

	 (= (time_cost) 0) 
	 (= (place_qual) 0)

	 ;;placing costs 
	 (= (place_succ cotnap1 A placevert) 2)
	 (= (place_succ cotnap1 A placediag) 2) 
	 (= (place_succ cotnap1 A placerot) 2) 
	 (= (place_succ cotnap1 B placevert) 5) 
	 (= (place_succ cotnap1 B placediag) 4) 
	 (= (place_succ cotnap1 B placerot) 4) 
	 (= (place_succ cotnap1 C placevert) 13) 
	 (= (place_succ cotnap1 C placediag) 1) 
	 (= (place_succ cotnap1 C placerot) 1) 

	 ;;piling costs 
 	 (= (place_succ cotnap2 A placevert) 9) 
	 (= (place_succ cotnap2 A placediag) 4) 
	 (= (place_succ cotnap2 A placerot) 3) 
	 (= (place_succ cotnap2 B placevert) 27) 
	 (= (place_succ cotnap2 B placediag) 4) 
	 (= (place_succ cotnap2 B placerot) 4) 
	 (= (place_succ cotnap2 C placevert) 26) 
	 (= (place_succ cotnap2 C placediag) 18) 
	 (= (place_succ cotnap2 C placerot) 29) 

	 (obj_grasp_class cotnap1 short B) 
	 (obj_grasp_class cotnap1 long C) 
	 (obj_grasp_class cotnap2 short B) 
	 (obj_grasp_class cotnap2 long C) 
)

(:goal (and (on cotnap1 table) (at_pose cotnap1 long) (on cotnap2 cotnap1)))

(:metric minimize (+ (* 1 (time_cost)) (* 2 (place_qual)))) 

 )