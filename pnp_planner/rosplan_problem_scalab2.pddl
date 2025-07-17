;; piles of more than 2 objects with different deformation classes depending on the cloth
;; tarda mucho en sacar un plan pero lo saca (aprox 50 segs)

(define (problem PICKNPLACEpileclass)
(:domain PICKNPLACEpileclass)
(:objects
    table towel waffle1 waffle2 checkered1 checkered2 checkered3 linen - garment
	placevert placediag placerot - placing
	long short - grasp
	grws rotws - workspace
	grasped placed notgrasped lifted - state
	home high_pose else drag_pose - position
	flat A B C - defclass
)

(:init (garment_state table placed) (known_obj towel) (garment_state towel notgrasped) (garment_at towel grws) (at_pose towel short) (not (corners_pos_known towel)) (defstate towel flat) ;;place object (cloth-to-table costs)
;;(:init (garment_state towel placed) (defstate waffle1 flat) (at_pose towel short) ;;pile object (cloth-to-cloth costs)
		
		(not (known_obj waffle1)) (garment_state waffle1 notgrasped) (garment_at waffle1 rotws) (at_pose waffle1 long) (not (corners_pos_known waffle1)) (defstate waffle1 flat) 
		(not (known_obj waffle2)) (garment_state waffle2 notgrasped) (garment_at waffle2 rotws) (at_pose waffle2 long) (not (corners_pos_known waffle2)) (defstate waffle2 flat) 
		(not (known_obj checkered1)) (garment_state checkered1 notgrasped) (garment_at checkered1 rotws) (at_pose checkered1 long) (not (corners_pos_known checkered1)) (defstate checkered1 flat) 
		(not (known_obj checkered2)) (garment_state checkered2 notgrasped) (garment_at checkered2 rotws) (at_pose checkered2 long) (not (corners_pos_known checkered2)) (defstate checkered2 flat) 
		(not (known_obj checkered3)) (garment_state checkered3 notgrasped) (garment_at checkered3 rotws) (at_pose checkered3 long) (not (corners_pos_known checkered3)) (defstate checkered3 flat) 
		(not (known_obj linen)) (garment_state linen notgrasped) (garment_at linen rotws) (at_pose linen long) (not (corners_pos_known linen)) (defstate linen flat) 

		(robot_at else) (robot_empty)
		(= (time_cost) 0)
		(= (place_qual) 0)

		;;placing costs
		(= (place_succ towel A placevert) 1)
		(= (place_succ towel A placediag) 1)
		(= (place_succ towel A placerot) 1)
		(= (place_succ towel B placevert) 1)
		(= (place_succ towel B placediag) 1)
		(= (place_succ towel B placerot) 1)
		(= (place_succ towel C placevert) 1)
		(= (place_succ towel C placediag) 1)
		(= (place_succ towel C placerot) 1)

		;;piling costs
		(= (place_succ waffle1 A placevert) 1)
		(= (place_succ waffle1 A placediag) 1)
		(= (place_succ waffle1 A placerot) 1)
		(= (place_succ waffle1 B placevert) 1)
		(= (place_succ waffle1 B placediag) 1)
		(= (place_succ waffle1 B placerot) 1)
		(= (place_succ waffle1 C placevert) 1)
		(= (place_succ waffle1 C placediag) 1)
		(= (place_succ waffle1 C placerot) 1)

		(= (place_succ waffle2 A placevert) 1)
		(= (place_succ waffle2 A placediag) 1)
		(= (place_succ waffle2 A placerot) 1)
		(= (place_succ waffle2 C placevert) 1)
		(= (place_succ waffle2 C placediag) 1)
		(= (place_succ waffle2 C placerot) 1)
		(= (place_succ waffle2 B placevert) 44)
		(= (place_succ waffle2 B placediag) 30)
		(= (place_succ waffle2 B placerot) 30)

		(= (place_succ checkered1 A placevert) 1)
		(= (place_succ checkered1 A placediag) 1)
		(= (place_succ checkered1 A placerot) 1)
		(= (place_succ checkered1 B placevert) 1)
		(= (place_succ checkered1 B placediag) 1)
		(= (place_succ checkered1 B placerot) 1)
		(= (place_succ checkered1 C placevert) 44)
		(= (place_succ checkered1 C placediag) 30)
		(= (place_succ checkered1 C placerot) 30)

		(= (place_succ checkered2 A placevert) 1)
		(= (place_succ checkered2 A placediag) 1)
		(= (place_succ checkered2 A placerot) 1)
		(= (place_succ checkered2 B placevert) 1)
		(= (place_succ checkered2 B placediag) 1)
		(= (place_succ checkered2 B placerot) 1)
		(= (place_succ checkered2 C placevert) 44)
		(= (place_succ checkered2 C placediag) 30)
		(= (place_succ checkered2 C placerot) 30)

		(= (place_succ checkered3 A placevert) 1)
		(= (place_succ checkered3 A placediag) 1)
		(= (place_succ checkered3 A placerot) 1)
		(= (place_succ checkered3 B placevert) 1)
		(= (place_succ checkered3 B placediag) 1)
		(= (place_succ checkered3 B placerot) 1)
		(= (place_succ checkered3 C placevert) 44)
		(= (place_succ checkered3 C placediag) 30)
		(= (place_succ checkered3 C placerot) 30)

		(= (place_succ linen A placevert) 1)
		(= (place_succ linen A placediag) 1)
		(= (place_succ linen A placerot) 1)
		(= (place_succ linen B placevert) 1)
		(= (place_succ linen B placediag) 1)
		(= (place_succ linen B placerot) 1)
		(= (place_succ linen C placevert) 44)
		(= (place_succ linen C placediag) 30)
		(= (place_succ linen C placerot) 30)

		(obj_grasp_class towel long A) ;;placed_obj grasped by multedges results in deformation class A
		(obj_grasp_class towel short A) ;; It can be the same as with long, the second object will adapt to the first object's init pose

		(obj_grasp_class waffle1 long A) 
		(obj_grasp_class waffle1 short B)

		(obj_grasp_class waffle2 long A) 
		(obj_grasp_class waffle2 short B)

		(obj_grasp_class checkered1 long C) 
		(obj_grasp_class checkered1 short B)

		(obj_grasp_class checkered2 long C) 
		(obj_grasp_class checkered2 short B)

		(obj_grasp_class checkered3 long C) 
		(obj_grasp_class checkered3 short B)

		(obj_grasp_class linen long C) 
		(obj_grasp_class linen short B)
)

;;(:goal (and (garment_state towel placed) ))
(:goal (and (garment_state towel placed) (on waffle1 towel) (on waffle2 waffle1) (on checkered1 waffle2)
))

;;(:metric minimize (time_cost))
;;(:metric minimize (place_qual))
(:metric minimize (+ (time_cost) (place_qual))) ;; to select placing actions based on simplicity (vertical, diagonal, rotating)
;;(:metric minimize (+ (* 5 (time_cost)) (* 10 (place_qual))))

)