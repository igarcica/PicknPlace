(define (problem PICKNPLACEpileclass) 
(:domain PICKNPLACEpileclass)
(:objects 
 	 table waffle1 checkered1 - garment 
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
	 (not (known_obj checkered1)) (garment_state checkered1 notgrasped) (garment_at checkered1 rotws) (at_pose checkered1 long) (not (corners_pos_known checkered1)) (defstate checkered1 flat) 

	 (= (time_cost) 0) 
	 (= (place_qual) 0) 

	 ;;placing costs 
	 (= (place_succ waffle1 A placevert) 2)
	 (= (place_succ waffle1 A placediag) 1) 
	 (= (place_succ waffle1 A placerot) 1) 
	 (= (place_succ waffle1 B placevert) 5) 
	 (= (place_succ waffle1 B placediag) 4) 
	 (= (place_succ waffle1 B placerot) 4) 
	 (= (place_succ waffle1 C placevert) 1) 
	 (= (place_succ waffle1 C placediag) 1) 
	 (= (place_succ waffle1 C placerot) 1) 

	 ;;piling costs 
 	 (= (place_succ checkered1 A placevert) 3) 
	 (= (place_succ checkered1 A placediag) 2) 
	 (= (place_succ checkered1 A placerot) 2) 
	 (= (place_succ checkered1 B placevert) 30) 
	 (= (place_succ checkered1 B placediag) 5) 
	 (= (place_succ checkered1 B placerot) 4) 
	 (= (place_succ checkered1 C placevert) 27) 
	 (= (place_succ checkered1 C placediag) 10) 
	 (= (place_succ checkered1 C placerot) 11) 

	 (obj_grasp_class waffle1 short A) 
	 (obj_grasp_class waffle1 long A) 
	 (obj_grasp_class checkered1 short B) 
	 (obj_grasp_class checkered1 long C) 
)

(:goal (and (on waffle1 table) (on checkered1 waffle1) ))

(:metric minimize (+ (* 1 (time_cost)) (* 10 (place_qual)))) 

 )