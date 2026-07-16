(define (problem PICKNPLACEpileclass) 
(:domain PICKNPLACEpileclass)
(:objects 
 	 table check waffle2 twlrag waffle1 - garment 
	 placevert placediag placerot - placing 
	 long short - grasp 
	 grws rotws - workspace 
	 grasped placed notgrasped lifted - state 
	 home high_pose else drag_pose - position 
	 flat A B C - defclass 
 ) 

(:init (garment_state table placed) (robot_at else) (robot_empty) 

	 ;; object to place (cloth-to-table costs) 
	 (known_obj check) (garment_state check notgrasped) (garment_at check rotws) (at_pose check short) (not (corners_pos_known check)) (defstate check flat) 

	 ;; objects to pile (cloth-to-cloth costs) 
	 (not (known_obj waffle2)) (garment_state waffle2 notgrasped) (garment_at waffle2 rotws) (at_pose waffle2 short) (not (corners_pos_known waffle2)) (defstate waffle2 flat) 
	 (not (known_obj twlrag)) (garment_state twlrag notgrasped) (garment_at twlrag rotws) (at_pose twlrag short) (not (corners_pos_known twlrag)) (defstate twlrag flat) 
	 (not (known_obj waffle1)) (garment_state waffle1 notgrasped) (garment_at waffle1 rotws) (at_pose waffle1 short) (not (corners_pos_known waffle1)) (defstate waffle1 flat) 

	 (= (time_cost) 0) 
	 (= (place_qual) 0) 

	 ;;placing costs 
	 (= (place_succ check A placevert) 2)
	 (= (place_succ check A placediag) 2) 
	 (= (place_succ check A placerot) 2) 
	 (= (place_succ check B placevert) 6) 
	 (= (place_succ check B placediag) 5) 
	 (= (place_succ check B placerot) 5) 
	 (= (place_succ check C placevert) 3) 
	 (= (place_succ check C placediag) 1) 
	 (= (place_succ check C placerot) 1) 

	 ;;piling costs 
 	 (= (place_succ waffle2 A placevert) 9) 
	 (= (place_succ waffle2 A placediag) 7) 
	 (= (place_succ waffle2 A placerot) 8) 
	 (= (place_succ waffle2 B placevert) 27) 
	 (= (place_succ waffle2 B placediag) 5) 
	 (= (place_succ waffle2 B placerot) 2) 
	 (= (place_succ waffle2 C placevert) 26) 
	 (= (place_succ waffle2 C placediag) 10) 
	 (= (place_succ waffle2 C placerot) 11) 

	 (= (place_succ twlrag A placevert) 9) 
	 (= (place_succ twlrag A placediag) 7)  
	 (= (place_succ twlrag A placerot) 8) 
	 (= (place_succ twlrag B placevert) 27) 
	 (= (place_succ twlrag B placediag) 5) 
	 (= (place_succ twlrag B placerot) 2) 
	 (= (place_succ twlrag C placevert) 26) 
	 (= (place_succ twlrag C placediag) 10) 
	 (= (place_succ twlrag C placerot) 11) 

	 (= (place_succ waffle1 A placevert) 9) 
	 (= (place_succ waffle1 A placediag) 7) 
	 (= (place_succ waffle1 A placerot) 8) 
	 (= (place_succ waffle1 B placevert) 27) 
	 (= (place_succ waffle1 B placediag) 5) 
	 (= (place_succ waffle1 B placerot) 2) 
	 (= (place_succ waffle1 C placevert) 26) 
	 (= (place_succ waffle1 C placediag) 10) 
	 (= (place_succ waffle1 C placerot) 11) 

	 (obj_grasp_class check short A) 
	 (obj_grasp_class check long A) 
	 (obj_grasp_class waffle2 short A) 
	 (obj_grasp_class waffle2 long A) 
	 (obj_grasp_class twlrag short A) 
	 (obj_grasp_class twlrag long A) 
	 (obj_grasp_class waffle1 short A) 
	 (obj_grasp_class waffle1 long A) 
)

(:goal (and (on check table) (on twlrag check) (on waffle1 twlrag) (on waffle2 waffle1) (at_pose twlrag long)  (at_pose waffle1 long)  (at_pose waffle2 long)))

(:metric minimize (+ (* 1 (time_cost)) (* 2 (place_qual)))) 

 )