(define (domain PICKNPLACEtest)

(:requirements :strips :typing :fluents :disjunctive-preconditions :negative-preconditions :durative-actions)

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
)

;; Move to any waypoint, avoiding terrain
(:durative-action check_corners
	:parameters (?cloth - garment)
	:duration ( = ?duration 1)
	:condition (and
				(over all (garment_obj ?cloth))
				(at start (garment_state notgrasped))
				(at start (corners_pos unknown)))
	:effect (and
			(at start (not (corners_pos unknown)))
			(at end (corners_pos known))  
			(at end (increase (time_cost) 0)))
)

(:durative-action home
:parameters (?cloth - garment)
	:duration ( = ?duration 1)
	:condition (and
				(over all (garment_obj ?cloth))
				(at start (corners_pos known))
				(at start (robot_at postgrasp)))
	:effect (and
			(at end (not (robot_at postgrasp)))
			(at end (robot_at home))  
			(at end (increase (time_cost) 0)))
)

(:durative-action grasp
	:parameters (?cloth - garment ?ws - workspace ?gr - grasp)
	:duration ( = ?duration 60)
	:condition (and
				(over all (garment_obj ?cloth))
				(at start (robot_at home))
				(at start (garment_at ?ws))
				(at start (at_pose ?gr))
				(at start (garment_state notgrasped)))
	:effect (and
			(at start (not (garment_state notgrasped)))
			(at end (not (robot_at home)))
			(at end (garment_state grasped))  
			(at end (increase (time_cost) 1)))
)

(:durative-action drag
	:parameters (?cloth - garment ?initws ?endws - workspace)
	:duration ( = ?duration 1)
	:condition (and 
				(over all (garment_obj ?cloth))
				(at start (garment_at ?initws))
				(at start (garment_state notgrasped)))
	:effect (and 
			(at end (garment_at ?endws))
			(at start (not (garment_at ?initws)))
			(at end (increase (time_cost) 1)))
)

(:durative-action rotate
	:parameters (?cloth - garment ?initedge ?endedge - grasp)
	:duration ( = ?duration 1)
	:condition (and 
				(over all (garment_obj ?cloth))
				(at start (garment_at grrotws))
				(at start (at_pose ?initedge))
				(at start (garment_state notgrasped)))
	:effect (and 
			(at end (at_pose ?endedge))
			(at start (not (at_pose ?initedge)))
			(at end (increase (time_cost) 1)))
)

(:durative-action check_deformation
	:parameters (?cloth - garment)
	:duration ( = ?duration 1)
	:condition (and 
				(over all (garment_obj ?cloth))
				(at start (garment_state grasped)))
	:effect (and 
			(at end (garment_state lifted))
			(at start (not (garment_state grasped)))
			(at end (increase (time_cost) 1)))
)

(:durative-action placevert
	:parameters (?cloth - garment)
	:duration ( = ?duration 1)
	:condition (and 
				(over all (garment_obj ?cloth))
				(at start (garment_state lifted)))
	:effect (and 
			(at end (garment_state placed))
			(at start (not (garment_state lifted)))
			(at end (increase (time_cost) 1)))
)

;;(:durative-action placediag
;;	:parameters (?cloth - garment)
;;	:duration ( = ?duration 2)
;;	:condition (and 
;;				(over all (garment_obj ?cloth))
;;				(at start (garment_state lifted)))
;;	:effect (and 
;;			(at end (garment_state placed))
;;			(at start (not (garment_state lifted)))
;;			(at end (increase (time_cost) 2)))
;;)

)