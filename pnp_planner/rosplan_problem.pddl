(define (problem PICKNPLACEpileclass) 
(:domain PICKNPLACEpileclass)
(:objects 
 	 table twlrag waffle1 checkered1 - garment 
	 placevert placediag placerot - placing 
	 long short - grasp 
	 grws rotws - workspace 
	 grasped placed notgrasped lifted - state 
	 home high_pose else drag_pose - position 
	 flat A B C - defclass 
 ) 

(:init (garment_state table placed) (robot_at else) (robot_empty) 

	 ;; object to place (cloth-to-table costs) 
	 (known_obj twlrag) (garment_state twlrag notgrasped) (garment_at twlrag rotws) (at_pose twlrag long) (not (corners_pos_known twlrag)) (defstate twlrag flat) 

	 ;; objects to pile (cloth-to-cloth costs) 
	 (not (known_obj waffle1)) (garment_state waffle1 notgrasped) (garment_at waffle1 rotws) (at_pose waffle1 long) (not (corners_pos_known waffle1)) (defstate waffle1 flat) 
	 (not (known_obj checkered1)) (garment_state checkered1 notgrasped) (garment_at checkered1 rotws) (at_pose checkered1 long) (not (corners_pos_known checkered1)) (defstate checkered1 flat) 

	 (= (time_cost) 0) 
	 (= (place_qual) 0) 

	 ;;placing costs 
	 (= (place_succ twlrag A placevert) 17)
	 (= (place_succ twlrag A placediag) 1) 
	 (= (place_succ twlrag A placerot) 1) 
	 (= (place_succ twlrag B placevert) 8) 
	 (= (place_succ twlrag B placediag) 22) 
	 (= (place_succ twlrag B placerot) 6) 
	 (= (place_succ twlrag C placevert) 30) 
	 (= (place_succ twlrag C placediag) 25) 
	 (= (place_succ twlrag C placerot) 6) 

	 ;;piling costs 
 	 (= (place_succ waffle1 A placevert) 7) 
	 (= (place_succ waffle1 A placediag) 3) 
	 (= (place_succ waffle1 A placerot) 4) 
	 (= (place_succ waffle1 B placevert) 30) 
	 (= (place_succ waffle1 B placediag) 14) 
	 (= (place_succ waffle1 B placerot) 9) 
	 (= (place_succ waffle1 C placevert) 30) 
	 (= (place_succ waffle1 C placediag) 30) 
	 (= (place_succ waffle1 C placerot) 30) 

	 (= (place_succ checkered1 A placevert) 7) 
	 (= (place_succ checkered1 A placediag) 3) 
	 (= (place_succ checkered1 A placerot) 4) 
	 (= (place_succ checkered1 B placevert) 30) 
	 (= (place_succ checkered1 B placediag) 14) 
	 (= (place_succ checkered1 B placerot) 9) 
	 (= (place_succ checkered1 C placevert) 30) 
	 (= (place_succ checkered1 C placediag) 30) 
	 (= (place_succ checkered1 C placerot) 30) 

	 (obj_grasp_class twlrag short A) 
	 (obj_grasp_class twlrag short A) 
	 (obj_grasp_class waffle1 short A) 
	 (obj_grasp_class waffle1 short A) 
	 (obj_grasp_class checkered1 short A) 
	 (obj_grasp_class checkered1 short A) 
)

(:goal (and (on twlrag table) (on waffle1 twlrag) (on checkered1 waffle1) ))

(:metric minimize (+ (time_cost) (place_qual))) 

 )