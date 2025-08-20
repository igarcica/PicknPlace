(define (problem PICKNPLACEpileclass) 
(:domain PICKNPLACEpileclass)
(:objects 
 	 table checkered1 checkered2 - garment 
	 placevert placediag placerot - placing 
	 long short - grasp 
	 grws rotws - workspace 
	 grasped placed notgrasped lifted - state 
	 home high_pose else drag_pose - position 
	 flat A B C - defclass 
 ) 

(:init (garment_state table placed) (robot_at else) (robot_empty) 

	 ;; object to place (cloth-to-table costs) 
	 (known_obj checkered1) (garment_state checkered1 notgrasped) (garment_at checkered1 rotws) (at_pose checkered1 long) (not (corners_pos_known checkered1)) (defstate checkered1 flat) 

	 ;; objects to pile (cloth-to-cloth costs) 
	 (not (known_obj checkered2)) (garment_state checkered2 notgrasped) (garment_at checkered2 rotws) (at_pose checkered2 long) (not (corners_pos_known checkered2)) (defstate checkered2 flat) 

	 ;; To plan placement of second object
	 ;;(garment_state checkered2 notgrasped) (garment_at checkered2 rotws) (at_pose checkered2 short) (not (corners_pos_known checkered2)) (defstate checkered2 flat) 
	 ;;(known_obj checkered2) (known_obj checkered1) (garment_state checkered1 placed) (at_pose checkered1 short) (on checkered1 table)

	 (= (time_cost) 0) 
	 (= (place_qual) 0) 

	 ;;placing costs 
	 (= (place_succ checkered1 A placevert) 0)
	 (= (place_succ checkered1 A placediag) 0) 
	 (= (place_succ checkered1 A placerot) 0) 
	 (= (place_succ checkered1 B placevert) 30) 
	 (= (place_succ checkered1 B placediag) 0) 
	 (= (place_succ checkered1 B placerot) 30) 
	 (= (place_succ checkered1 C placevert) 30) 
	 (= (place_succ checkered1 C placediag) 30) 
	 (= (place_succ checkered1 C placerot) 30) 

	 ;;piling costs 
 	 (= (place_succ checkered2 A placevert) 0) 
	 (= (place_succ checkered2 A placediag) 0) 
	 (= (place_succ checkered2 A placerot) 0) 
	 (= (place_succ checkered2 B placevert) 30) 
	 (= (place_succ checkered2 B placediag) 0) 
	 (= (place_succ checkered2 B placerot) 30) 
	 (= (place_succ checkered2 C placevert) 30) 
	 (= (place_succ checkered2 C placediag) 30) 
	 (= (place_succ checkered2 C placerot) 30) 

	 (obj_grasp_class checkered1 short B) 
	 (obj_grasp_class checkered1 long C) 
	 (obj_grasp_class checkered2 short B) 
	 (obj_grasp_class checkered2 long C) 
)

(:goal (and (on checkered1 table) (on checkered2 checkered1) ))

(:metric minimize (+ (time_cost) (* 3 (place_qual)))) 

 )