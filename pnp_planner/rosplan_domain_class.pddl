;;Takes into account functions. Used with Metric-ff.
;;Takes into account rotate and drag in the plan to change grasped edge for minimizing cost
;;Place success cost depends on deformation class and deformation class depends on grasped edge

(define (domain PICKNPLACEtest)

;;(:requirements :strips :typing :disjunctive-preconditions :negative-preconditions :durative-actions :numeric-fluents)
(:requirements :fluents)

(:types
	garment 
	grasp
    placing
    workspace
	state
	corners
	position
	defclass
)

(:predicates
	(garment_obj ?cloth - garment)
	(garment_at ?ws - workspace)
	(at_pose ?edge - grasp)
	(garment_state ?state - state)
	(corners_pos ?corner - corners)
	(robot_at ?pos - position)
	(defstate ?class - defclass) 		;;the object has deformation class ?class
	(obj_grasp_class ?cloth - garment ?edge - grasp ?class - defclass) ;; ?cloth grasped by ?edge will produce deformation class ?class
)

(:functions
    (time_cost)
	(place_qual)
	(place_succ ?class - defclass ?place - placing)
)

;; Move to any waypoint, avoiding terrain
(:action check_corners
	:parameters (?cloth - garment)
	:precondition (and
				(garment_obj ?cloth)
				(garment_state notgrasped)
				(corners_pos unknown)
				(robot_at high_pose))
	:effect (and
			(not (corners_pos unknown))
			(corners_pos known)
			(increase (time_cost) 1)
			(increase (place_qual) 0))
)

(:action home
	:parameters (?cloth - garment)
	:precondition (and
				(garment_obj ?cloth)
				)
	:effect (and
			(robot_at home)
			(not (robot_at high_pose))
			(increase (time_cost) 1)
			(increase (place_qual) 0))
)

(:action go_high
	:parameters (?cloth - garment)
	:precondition (and
				(garment_obj ?cloth)
				(corners_pos unknown)
				(robot_at home)
				)
	:effect (and
			(not (robot_at home))
			(robot_at high_pose)
			(increase (time_cost) 1)
			(increase (place_qual) 0))
)

(:action grasp
	:parameters (?cloth - garment ?ws - workspace ?gr - grasp ?class - defclass)
	:precondition (and
				(garment_obj ?cloth)
				(robot_at home)
				;(garment_at ?ws)
				(at_pose ?gr)
				(garment_state notgrasped)
				(corners_pos known)
				(obj_grasp_class ?cloth ?gr ?class)
				(defstate flat))
	:effect (and
			(not (garment_state notgrasped))
			(not (robot_at home))
			;(robot_at grasp_pose)
			(garment_state grasped)
			(defstate ?class)
			(increase (time_cost) 1)
			(increase (place_qual) 0))
)

(:action drag
	:parameters (?cloth - garment)
	:precondition (and 
				(garment_obj ?cloth)
				(robot_at home)
				(corners_pos known)
				(garment_at grws)
				(garment_state notgrasped))
	:effect (and 
			(not (garment_at grws))
			(garment_at rotws)
			;(not (robot_at home))
			;(robot_at drag_pose)
			(increase (time_cost) 1)
			(increase (place_qual) 0))
)

(:action rotate
	:parameters (?cloth - garment ?initedge ?endedge - grasp)
	:precondition (and 
				(garment_obj ?cloth)
				;(not (robot_at high_pose)) ;Should be better at home or at drag_pose
				;(or (robot_at home) (robot_at drag_pose))
				(robot_at home)
				(corners_pos known)
				(garment_at rotws)
				(at_pose ?initedge)
				(garment_state notgrasped))
	:effect (and 
			(not (at_pose ?initedge))
			(at_pose ?endedge)
			(not (corners_pos known))
			(corners_pos unknown)
			(not (robot_at home))
			;(robot_at drag_pose)
			(increase (time_cost) 0)
			(increase (place_qual) 0))
)

(:action check_deformation
	:parameters (?cloth - garment)
	:precondition (and 
				(garment_obj ?cloth)
				(garment_state grasped))
	:effect (and 
			(garment_state lifted)
			(not (garment_state grasped))
			(increase (time_cost) 1)
			(increase (place_qual) 0))
)

(:action placevert
	:parameters (?cloth - garment ?edge - grasp ?class - defclass)
	:precondition (and 
				(garment_obj ?cloth)
				(at_pose ?edge)
				(garment_state lifted)
				(defstate ?class))
	:effect (and 
			(garment_state placed)
			(not (garment_state lifted))
			(increase (time_cost) 1)
			(increase (place_qual) (place_succ ?class placevert)))
)

(:action placediag
	:parameters (?cloth - garment ?edge - grasp ?class - defclass)
	:precondition (and
				(garment_obj ?cloth)
				(at_pose ?edge)
				(garment_state lifted)
				(defstate ?class))
	:effect (and 
			(garment_state placed)
			(not (garment_state lifted))
			(increase (time_cost) 1)
			(increase (place_qual) (place_succ ?class placediag)))
)

)