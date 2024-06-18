;;Takes into account functions. Used with Metric-ff.
;;Problem: Does not include in the plan rotate and drag, probably because it prioritizes less number of actions.
;;Solution: Seems that it works with Metric-FF -o rosplan_domain.pddl -f rosplan_problem.pddl -s 3 -w 1. However, it also adds drag although it is not necessary

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
	(garment_obj ?cloth - garment)
	(garment_at ?wp - workspace)
	(at_pose ?edge - grasp)
	(garment_state ?state - state)
	(corners_pos ?corner - corners)
	(robot_at ?pos - position)
)

(:functions
    (time_cost)
	(place_qual)
	(place_succ ?cloth - garment ?edge - grasp ?place - placing)
)

;; Move to any waypoint, avoiding terrain
(:action check_corners
	:parameters (?cloth - garment)
	:precondition (and
				(garment_obj ?cloth)
				(garment_state notgrasped)
				(corners_pos unknown))
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
				(corners_pos known)
				(robot_at postgrasp))
	:effect (and
			(not (robot_at postgrasp))
			(robot_at home)
			(increase (time_cost) 1)
			(increase (place_qual) 0))
)

(:action grasp
;;	:parameters (?cloth - garment ?ws - workspace ?gr - grasp)
	:parameters (?cloth - garment ?gr - grasp)
	:precondition (and
				(garment_obj ?cloth)
				(robot_at home)
;;				(garment_at ?ws)
				(at_pose ?gr)
				(garment_state notgrasped))
	:effect (and
			(not (garment_state notgrasped))
			(not (robot_at home))
			(garment_state grasped)
			(increase (time_cost) 1)
			(increase (place_qual) 0))
)

(:action drag
;;	:parameters (?cloth - garment ?initws ?endws - workspace)
	:parameters (?cloth - garment)
	:precondition (and 
				(garment_obj ?cloth)
				(garment_at grws)
				(garment_state notgrasped))
	:effect (and 
			(not (garment_at grws))
			(garment_at grrotws)
			(increase (time_cost) 1)
			(increase (place_qual) 0))
)

(:action rotate
	:parameters (?cloth - garment ?initedge ?endedge - grasp)
	:precondition (and 
				(garment_obj ?cloth)
				(garment_at grrotws)
				(at_pose ?initedge)
				(garment_state notgrasped))
	:effect (and 
			(not (at_pose ?initedge))
			(at_pose ?endedge)
			(increase (time_cost) 1)
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
	:parameters (?cloth - garment ?edge - grasp)
	:precondition (and 
				(garment_obj ?cloth)
				(at_pose ?edge)
				(garment_state lifted))
	:effect (and 
			(garment_state placed)
			(not (garment_state lifted))
			(increase (time_cost) 1)
			(increase (place_qual) (place_succ ?cloth ?edge placevert)))
)

(:action placediag
	:parameters (?cloth - garment ?edge - grasp)
	:precondition (and
				(garment_obj ?cloth)
				(at_pose ?edge)
				(garment_state lifted))
	:effect (and 
			(garment_state placed)
			(not (garment_state lifted))
			(increase (time_cost) 1)
			(increase (place_qual) (place_succ ?cloth ?edge placediag)))
)

)