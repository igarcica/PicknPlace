(define (problem PICKNPLACEpileclass) 
(:domain PICKNPLACEpileclass)
(:objects 
 	 table waffle1 waffle2 - garment 
	 placevert placediag placerot - placing 
	 long short - grasp 
	 grws rotws - workspace 
	 grasped placed notgrasped lifted - state 
	 home high_pose else drag_pose - position 
	 flat A B C - defclass 
 ) 

(:init (garment_state table placed) (robot_at else) (robot_empty) 

	 ;; object to place (cloth-to-table costs) 
	 (known_obj waffle1) (garment_state waffle1 notgrasped) (garment_at waffle1 rotws) (at_pose waffle1 long) (not (corners_pos_known waffle1)) (defstate waffle1 flat) 

	 ;; objects to pile (cloth-to-cloth costs) 
	 (not (known_obj waffle2)) (garment_state waffle2 notgrasped) (garment_at waffle2 rotws) (at_pose waffle2 long) (not (corners_pos_known waffle2)) (defstate waffle2 flat) 

	 (= (time_cost) 0) 
	 (= (place_qual) 0) 

	 ;;placing costs 
	 (= (place_succ waffle1 A placevert) 1)
	 (= (place_succ waffle1 A placediag) 3) 
	 (= (place_succ waffle1 A placerot) 1) 
	 (= (place_succ waffle1 B placevert) 5) 
	 (= (place_succ waffle1 B placediag) 4) 
	 (= (place_succ waffle1 B placerot) 4) 
	 (= (place_succ waffle1 C placevert) 1) 
	 (= (place_succ waffle1 C placediag) 1) 
	 (= (place_succ waffle1 C placerot) 1) 

	 ;;piling costs 
 	 (= (place_succ waffle2 A placevert) 3) 
	 (= (place_succ waffle2 A placediag) 4) 
	 (= (place_succ waffle2 A placerot) 3) 
	 (= (place_succ waffle2 B placevert) 30) 
	 (= (place_succ waffle2 B placediag) 4) 
	 (= (place_succ waffle2 B placerot) 4) 
	 (= (place_succ waffle2 C placevert) 27) 
	 (= (place_succ waffle2 C placediag) 10) 
	 (= (place_succ waffle2 C placerot) 11) 

	 (obj_grasp_class waffle1 short A) 
	 (obj_grasp_class waffle1 long A) 
	 (obj_grasp_class waffle2 short B) 
	 (obj_grasp_class waffle2 long C) 
)

(:goal (and (on waffle1 table) (on waffle2 waffle1) ))

(:metric minimize (+ (* 1 (time_cost)) (* 10 (place_qual)))) 

 )