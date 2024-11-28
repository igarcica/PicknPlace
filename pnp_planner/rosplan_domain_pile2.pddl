;; WORKS - simple place succ + Considers two objects, being the second object's pose unknown 

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
)

(:predicates
	(garment_at ?cloth - garment ?ws - workspace)
	(at_pose ?cloth - garment ?edge - grasp)
	(garment_state ?cloth - garment ?state - state)
	(corners_pos_known ?cloth - garment)
	(robot_at ?pos - position)
	(robot_empty)
	(known_obj ?cloth - garment)
)

(:functions
    (time_cost)
	(place_qual)
	(place_succ ?cloth - garment ?edge - grasp ?place - placing)
)

;; It should consider ?edge and ?ws such that place_succ is the worst (just in case) - HOW??
;; Later, once the first object is placed the ?edge and ?ws should be updated with real data - POSSIBLE?
(:action new_object
	:parameters (?edge - grasp ?ws - workspace)
	:precondition (and
				(garment_state towel placed)
				(not (known_obj hola)))
	:effect (and
			(known_obj hola)
			(garment_at hola ?ws) 
			(at_pose hola ?edge) 
			(garment_state hola notgrasped)
			(not (corners_pos_known hola))
			(increase (time_cost) 0)
			(increase (place_qual) 0))
)

;; Move to any waypoint, avoiding terrain
(:action check_corners
	:parameters (?cloth - garment)
	:precondition (and
				(garment_state ?cloth notgrasped)
				(not (corners_pos_known ?cloth))
				(robot_at high_pose))
	:effect (and
			(corners_pos_known ?cloth)
			(increase (time_cost) 1)
			(increase (place_qual) 0))
)

(:action home
	:precondition (and 
				(robot_empty)
				(or (robot_at else) (robot_at high_pose)))
	:effect (and
			(robot_at home)
			(not (robot_at high_pose))
			(not (robot_at else))
			(increase (time_cost) 1)
			(increase (place_qual) 0))
)

(:action go_high
	:precondition (and
				(robot_at home)
				)
	:effect (and
			(not (robot_at home))
			(robot_at high_pose)
			(increase (time_cost) 1)
			(increase (place_qual) 0))
)

(:action grasp
	:parameters (?cloth - garment ?ws - workspace ?gr - grasp)
	:precondition (and
				(robot_at home)
				(robot_empty)
				;(garment_at ?ws)
				(at_pose ?cloth ?gr)
				(garment_state ?cloth notgrasped)
				(corners_pos_known ?cloth))
	:effect (and
			(not (garment_state ?cloth notgrasped))
			(not (robot_at home))
			(robot_at else)
			;(robot_at grasp_pose)
			(not (robot_empty))
			(garment_state ?cloth grasped)
			(increase (time_cost) 1)
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
			(not (garment_at ?cloth grws))
			(garment_at ?cloth rotws)
			(robot_at drag_pose)
			(not (robot_at home))
			(increase (time_cost) 1)
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
			(increase (time_cost) 1)
			(increase (place_qual) 0))
)

(:action placevert
	:parameters (?cloth - garment ?edge - grasp)
	:precondition (and 
				(at_pose ?cloth ?edge)
				(garment_state ?cloth lifted))
	:effect (and 
			(robot_empty)
			(garment_state ?cloth placed)
			(not (garment_state ?cloth lifted))
			(increase (time_cost) 1)
			(increase (place_qual) (place_succ ?cloth ?edge placevert)))
)

(:action placediag
	:parameters (?cloth - garment ?edge - grasp)
	:precondition (and
				(at_pose ?cloth ?edge)
				(garment_state ?cloth lifted))
	:effect (and 
			(robot_empty)
			(garment_state ?cloth placed)
			(not (garment_state ?cloth lifted))
			(increase (time_cost) 1)
			(increase (place_qual) (place_succ ?cloth ?edge placediag)))
)

)