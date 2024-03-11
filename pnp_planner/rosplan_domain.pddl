(define (domain PICKNPLACEtest)

(:requirements :strips )

(:types
	garment 
	grasp
    placing
    workspace
	state
)

(:predicates
	(garment-at ?loth - garment ?ws - workspace)
    (at-garment ?cloth - garment)
    (at-pose ?edge - grasp)
	(garment-state ?state - state)
	(PP) (Lifted) (Placed)
)
(:functions (time-cost))
 

(:action Grasp
	:parameters (?cloth - garment ?ws - workspace ?edge - grasp)
	:precondition (and (garment-at ?cloth ?ws) (at-pose ?edge)  (garment-state notgrasped))
	:effect (and (garment-state grasped)  (not (garment-state notgrasped))
			(increase (time-cost) 1)
		))

(:action Approach
	:parameters (?cloth - garment ?edge - grasp ?initws ?endws - workspace)
	:precondition (and (garment-at ?cloth ?initws) (at-pose ?edge) (garment-state notgrasped) )
	:effect (and (garment-at ?cloth ?endws) 
			(increase (time-cost) 1)
		))

(:action Rotate
	:parameters (?cloth - garment ?initedge ?endedge - grasp ?ws - workspace)
	:precondition (and (garment-at ?cloth grrotws) (at-pose ?initedge) (garment-state notgrasped))
	:effect (and (at-pose ?endedge) (not (at-pose ?initedge))
			(increase (time-cost) 1)
	))

(:action Lift
	:parameters (?edge - grasp)
	:precondition (and (garment-state grasped))
	:effect (and (garment-state lifted) (not (garment-state grasped))
			(increase (time-cost) 1)
	))

(:action Release
	:parameters (?cloth - garment ?edge - grasp)
	:precondition (and (garment-state grasped))
	:effect (and (garment-state notgrasped) (not (garment-state grasped))
			(increase (time-cost) 1)
			))

(:action PlaceVert
	:parameters (?cloth - garment ?edge - grasp)
	:precondition (and (garment-state lifted))
	:effect (and (garment-state placed) (not (garment-state lifted))
			(increase (time-cost) 1000)
			))

(:action PlaceDiag
	:parameters (?cloth - garment ?edge - grasp)
	:precondition (and (garment-state lifted))
	:effect (and (garment-state placed) (not (garment-state lifted))
			(increase (time-cost) 1)
			))

)
