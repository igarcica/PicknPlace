(define (problem PICKNPLACEpileclass) 
(:domain PICKNPLACEpileclass)
(:objects 
 	 table towel pillowc - garment 
	 placevert placediag placerot - placing 
	 long short - grasp 
	 grws rotws - workspace 
	 grasped placed notgrasped lifted - state 
	 home high_pose else drag_pose - position 
	 flat A B C - defclass 
 ) 

(:init (garment_state table placed) (robot_at else) (robot_empty) 

	 ;; object to place (cloth-to-table costs) 
	 (known_obj towel) (garment_state towel notgrasped) (garment_at towel grws) (at_pose towel long) (not (corners_pos_known towel)) (defstate towel flat) 

	 ;; objects to pile (cloth-to-cloth costs) 
	 (not (known_obj pillowc)) (garment_state pillowc notgrasped) (garment_at pillowc rotws) (at_pose pillowc long) (not (corners_pos_known pillowc)) (defstate pillowc flat) 

	 (= (time_cost) 0) 
	 (= (place_qual) 0)

	 ;;placing costs 
	 (= (place_succ towel A placevert) 2)
	 (= (place_succ towel A placediag) 2) 
	 (= (place_succ towel A placerot) 2) 
	 (= (place_succ towel B placevert) 5) 
	 (= (place_succ towel B placediag) 5) 
	 (= (place_succ towel B placerot) 6) 
	 (= (place_succ towel C placevert) 1) 
	 (= (place_succ towel C placediag) 1) 
	 (= (place_succ towel C placerot) 1) 

	 ;;piling costs 
 	 (= (place_succ pillowc A placevert) 9) 
	 (= (place_succ pillowc A placediag) 4) 
	 (= (place_succ pillowc A placerot) 3) 
	 (= (place_succ pillowc B placevert) 27) 
	 (= (place_succ pillowc B placediag) 5) 
	 (= (place_succ pillowc B placerot) 3) 
	 (= (place_succ pillowc C placevert) 26) 
	 (= (place_succ pillowc C placediag) 10) 
	 (= (place_succ pillowc C placerot) 11) 

	 (obj_grasp_class towel short A) 
	 (obj_grasp_class towel long A) 
	 (obj_grasp_class pillowc short B) 
	 (obj_grasp_class pillowc long C) 
)

(:goal (and (on towel table) (on pillowc towel) ))

(:metric minimize (+ (* 1 (time_cost)) (* 2 (place_qual)))) 

 )