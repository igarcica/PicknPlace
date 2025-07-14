
(define (domain PICKNPLACEpileclass)

;;(:requirements :strips :typing :disjunctive-preconditions :negative-preconditions :durative-actions :numeric-fluents)
(:requirements :fluents)

(:types
	garment 
	grasp
    placing
    workspace
	state
	position
	defclass
)

(:predicates
	(garment_at ?cloth - garment ?ws - workspace)
	(at_pose ?cloth - garment ?edge - grasp)
	(garment_state ?cloth - garment ?state - state)
	(grasped_by ?cloth - garment ?edge - grasp)
	(corners_pos_known ?cloth - garment)
	(robot_at ?pos - position)
	(robot_empty)
	(known_obj ?cloth - garment)
	(defstate ?cloth - garment ?class - defclass) 		;;the object has deformation class ?class
	;;(obj_grasp_class ?cloth - garment ?edge - grasp ?class - defclass) ;; ?cloth grasped by ?edge will produce deformation class ?class
	(on ?piledcloth ?placedcloth - garment)
	(def_class ?cloth - garment ?class - defclass)
	(obj_grasp_class ?edge - grasp ?class - defclass) ;;?cloth grasped by ?edge will produce deformation class ?class
)

(:functions
    (time_cost)
	(place_qual)
	(place_succ ?cloth - garment ?class - defclass ?place - placing)
)

;; It should consider ?edge and ?ws such that place_succ is the worst (just in case) - HOW??
;; Later, once the first object is placed the ?edge and ?ws should be updated with real data - POSSIBLE?
(:action new_object
	:parameters (?objtopile ?placed_obj - garment)
	:precondition (and
				(garment_state ?placed_obj placed)
				(not (known_obj ?objtopile))
				(robot_at high_pose))
	:effect (and
			(known_obj ?objtopile) 
			(garment_state ?objtopile notgrasped)
			(not (corners_pos_known ?objtopile))
			(defstate ?objtopile flat)
			(increase (time_cost) 0)
			(increase (place_qual) 0))
)

;; Move to any waypoint, avoiding terrain
(:action check_corners
	:parameters (?cloth - garment ?ws - workspace ?edge - grasp)
	:precondition (and
				(garment_state ?cloth notgrasped)
				(not (corners_pos_known ?cloth))
				(robot_at high_pose))
	:effect (and
			(corners_pos_known ?cloth)
			(at_pose ?cloth ?edge)
			(garment_at ?cloth ?ws)
			(increase (time_cost) 0)
			(increase (place_qual) 0))
)

(:action home
	:parameters ()
	:precondition (and 
				(robot_empty)
				(or (robot_at else) (robot_at high_pose)))
	:effect (and
			(robot_at home)
			(not (robot_at high_pose))
			(not (robot_at else))
			(increase (time_cost) 0)
			(increase (place_qual) 0))
)

(:action go_high
	:parameters ()
	:precondition (and
				(robot_at home))
	:effect (and
			(not (robot_at home))
			(robot_at high_pose)
			(increase (time_cost) 0)
			(increase (place_qual) 1))
)

(:action grasp
	:parameters (?cloth - garment ?edge - grasp ?class - defclass)
	:precondition (and
				(robot_at home)
				(robot_empty)
				(at_pose ?cloth ?edge)
				(garment_state ?cloth notgrasped)
				(corners_pos_known ?cloth)
				;;(obj_grasp_class ?cloth ?edge ?class)
				;;(def_class ?cloth ?class)
				(defstate ?cloth flat)
				(obj_grasp_class ?edge ?class))
	:effect (and
			(not (garment_state ?cloth notgrasped))
			(not (robot_at home))
			(robot_at else)
			(not (robot_empty))
			(garment_state ?cloth grasped)
			(grasped_by ?cloth ?edge)
			(not (defstate ?cloth flat))
			(defstate ?cloth ?class)
			(increase (time_cost) 0)
			(increase (place_qual) 0))
)

(:action drag
	:parameters (?cloth - garment)
	:precondition (and 
				(robot_at home)
				(robot_empty)
				(corners_pos_known ?cloth)
				(garment_at ?cloth grws)
				(garment_state ?cloth notgrasped))
	:effect (and 
			;;(not (garment_at ?cloth grws))
			(not (corners_pos_known ?cloth))
			;;(garment_at ?cloth rotws)
			;;(robot_at drag_pose)
			(not (garment_at ?cloth grws)) ;;Change ws
			(garment_at ?cloth rotws)
			(robot_at home)
			(not (robot_at home))
			(increase (time_cost) 0)
			(increase (place_qual) 0))
)

(:action rotate
	:parameters (?cloth - garment ?initedge ?endedge - grasp)
	:precondition (and 
				(or (robot_at home) (robot_at drag_pose))
				(robot_empty)
				(corners_pos_known ?cloth)
				(garment_at ?cloth rotws)
				(at_pose ?cloth ?initedge)
				(garment_state ?cloth notgrasped))
	:effect (and 
			(not (at_pose ?cloth ?initedge)) ;;Change cloth pose
			(at_pose ?cloth ?endedge)
			(not (corners_pos_known ?cloth)) ;;As pose has changed, corners have to be detected again
			(robot_at else)
			(not (robot_at home))
			(not (robot_at drag_pose))
			;(not (robot_at home))
			;(robot_at drag_pose)
			(increase (time_cost) 0)
			(increase (place_qual) 0))
)

(:action check_deformation
	:parameters (?cloth - garment)
	:precondition (and 
				(garment_state ?cloth grasped))
	:effect (and 
			(garment_state ?cloth lifted)
			(not (garment_state ?cloth grasped))
			(increase (time_cost) 0)
			(increase (place_qual) 0))
)



(:action placediag
	:parameters (?cloth ?placedcloth - garment ?edge - grasp  ?class - defclass)
	:precondition (and
				(at_pose ?placedcloth ?edge) ;;piledcloth is on placedcloth is in placed with edge
				(at_pose ?cloth ?edge)
				(garment_state ?cloth lifted)
				(grasped_by ?cloth ?edge)
				(defstate ?cloth ?class))
	:effect (and 
			(robot_empty)
			(on ?cloth ?placedcloth)
			(garment_state ?cloth placed)
			(not (garment_state ?cloth lifted))
			(increase (time_cost) 2)
			(increase (place_qual) (place_succ ?cloth ?class placediag)))
)

(:action placerot
	:parameters (?cloth ?placedcloth - garment ?edge - grasp  ?class - defclass)
	:precondition (and
				(at_pose ?placedcloth ?edge) ;;piledcloth is on placedcloth is in placed with edge
				(at_pose ?cloth ?edge)
				(garment_state ?cloth lifted)
				(grasped_by ?cloth ?edge)
				(defstate ?cloth ?class))
	:effect (and 
			(robot_empty)
			(on ?cloth ?placedcloth)
			(garment_state ?cloth placed)
			(not (garment_state ?cloth lifted))
			(increase (time_cost) 3)
			(increase (place_qual) (place_succ ?cloth ?class placerot)))
)
(:action placevert
	:parameters (?cloth ?placedcloth - garment ?edge - grasp ?class - defclass)
	:precondition (and 
				(at_pose ?placedcloth ?edge) ;;piledcloth is on placedcloth is in placed with edge
				(at_pose ?cloth ?edge)
				(garment_state ?cloth lifted)
				(grasped_by ?cloth ?edge)
				(defstate ?cloth ?class))
	:effect (and 
			(robot_empty)
			(on ?cloth ?placedcloth)
			(garment_state ?cloth placed)
			(not (garment_state ?cloth lifted))
			(increase (time_cost) 1)
			(increase (place_qual) (place_succ ?cloth ?class placevert)))
)

)