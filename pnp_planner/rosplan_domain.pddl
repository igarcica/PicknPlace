(define (domain PICKNPLACEtest)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
	garment 
	grasp
    placing
    workspace
	state
)

(:predicates
	(garment_obj ?cloth - garment)
	(garment_at ?wp - workspace)
	(at_pose ?edge - grasp)
	(garment_state ?state - state)
)

(:functions
    (time_cost)
)

;; Move to any waypoint, avoiding terrain
(:durative-action grasp
	:parameters (?cloth - garment ?ws - workspace ?gr - grasp)
	:duration ( = ?duration 60)
	:condition (and
				(over all (garment_obj ?cloth))
				(at start (garment_at ?ws))
				(at start (at_pose ?gr))
				(at start (garment_state notgrasped)))
	:effect (and
			(at start (not (garment_state notgrasped)))
			(at end (garment_state grasped))  
			(at end (increase (time_cost) 1)))
)

(:durative-action approach
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

(:durative-action lift
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