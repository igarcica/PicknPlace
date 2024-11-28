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
	(garment_at ?cloth - garment ?ws - workspace)
	(at_pose ?cloth - garment ?edge - grasp)
	(garment_state ?cloth - garment ?state - state)
	(garment_obj_state ?cloth - garment ?state - state)
	(corners_pos ?cloth - garment ?corner - corners)
	(robot_at ?pos - position)
	(defstate ?cloth - garment ?class - defclass) 		;;the object has deformation class ?class
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
				(garment_state ?cloth notgrasped)
				(corners_pos ?cloth unknown)
				(robot_at high_pose))
	:effect (and
			(not (corners_pos ?cloth unknown))
			(corners_pos ?cloth known)
			(increase (time_cost) 1)
			(increase (place_qual) 0))
)

(:action home
	:parameters (?cloth - garment)
	:precondition (or
				(robot_at else)
				(robot_at high_pose)
				)
	:effect (and
			(robot_at home)
			(not (robot_at high_pose))
			(not (robot_at else))
			(increase (time_cost) 1)
			(increase (place_qual) 0))
)

(:action go_high
	:parameters (?cloth - garment)
	:precondition (and
				(corners_pos ?cloth unknown)
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
				(robot_at home)
				;(garment_at ?ws)
				(at_pose ?cloth ?gr)
				(garment_state ?cloth notgrasped)
				(corners_pos ?cloth known)
				(obj_grasp_class ?cloth ?gr ?class)
				(defstate ?cloth flat))
	:effect (and
			(not (garment_state ?cloth notgrasped))
			(not (robot_at home))
			(robot_at else)
			(garment_state ?cloth grasped)
			(defstate ?cloth ?class)
			(increase (time_cost) 1)
			(increase (place_qual) 0))
)

(:action drag
	:parameters (?cloth - garment)
	:precondition (and 
				(robot_at home)
				(corners_pos ?cloth known)
				(garment_at ?cloth grws)
				(garment_state ?cloth notgrasped))
	:effect (and 
			(not (garment_at ?cloth grws))
			(garment_at ?cloth rotws)
			(robot_at else)
			;(robot_at drag_pose)
			(increase (time_cost) 1)
			(increase (place_qual) 0))
)

(:action rotate
	:parameters (?cloth - garment ?initedge ?endedge - grasp)
	:precondition (and 
				;(not (robot_at high_pose)) ;Should be better at home or at drag_pose
				(or (robot_at home) (robot_at else))
				;(robot_at home)
				(corners_pos ?cloth known)
				(garment_at ?cloth rotws)
				(at_pose ?cloth ?initedge)
				(garment_state ?cloth notgrasped))
	:effect (and 
			(not (at_pose ?cloth ?initedge))
			(at_pose ?cloth ?endedge)
			(not (corners_pos ?cloth known))
			(corners_pos ?cloth unknown)
			(robot_at else)
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
			(increase (time_cost) 1)
			(increase (place_qual) 0))
)

(:action placevert
	:parameters (?cloth - garment ?edge - grasp ?class - defclass)
	:precondition (and 
				(at_pose ?cloth ?edge)
				(garment_state ?cloth lifted)
				(defstate ?cloth ?class))
	:effect (and 
			(garment_state ?cloth placed)
			(not (garment_state ?cloth lifted))
			(increase (time_cost) 1)
			(increase (place_qual) (place_succ ?class placevert)))
)

(:action placediag
	:parameters (?cloth - garment ?edge - grasp ?class - defclass)
	:precondition (and
				(at_pose ?cloth ?edge)
				(garment_state ?cloth lifted)
				(defstate ?cloth ?class))
	:effect (and 
			(garment_state ?cloth placed)
			(not (garment_state ?cloth lifted))
			(increase (time_cost) 1)
			(increase (place_qual) (place_succ ?class placediag)))
)

)