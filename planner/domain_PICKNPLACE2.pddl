;; Goes through rotate3 having init MultEdgesUp and RotWS to have less place-succ
;; But if in the init there is not RotWS it does not rotate and gets more cost

(define (domain PICKNPLACE2)
	(:requirements :strips)
	(:types garment grasp placing)
	(:predicates (GrWS) (RotWS) (SingleEdge) (MultEdgesUp) (MultEdgesLeft) (MultEdgesDown) (MultEdgesRight) (ParallelOrient)
								(PP) (Lifted) (Placed) (Lifted2)
								(at-garment ?cloth - garment) (at-pose ?edge - grasp)) ;;(type-garment ?cloth - garment) (edge-pose ?edge - grasp) )
	(:functions (time-cost) (place-qual) (place-succ ?cloth - garment ?edge - grasp ?place - placing))


;; PICK ACTIONS - depends on garment type and initial position

	(:action Grasp
		:parameters (?cloth - garment ?edge - grasp)
		:precondition (and (GrWS) (not (PP)) (at-garment ?cloth) (at-pose ?edge))
		:effect (and (PP)
			(increase (time-cost) 1)
			(increase (place-qual) 0)
			))

	(:action Approach
		:parameters (?cloth - garment ?edge - grasp)
		:precondition (and (GrWS) (not (PP)) (not (RotWS)) (at-garment ?cloth) (at-pose ?edge))
		:effect (and (RotWS)
			(increase (time-cost) 1)
			(increase (place-qual) 0)
			))

	(:action Rotate
		:parameters (?cloth - garment ?initedge ?endedge - grasp)
		;;:precondition (and (RotWS) (not (PP)))
		:precondition (and (RotWS) (not (PP)) (not (Placed)) (at-garment ?cloth) (at-pose ?initedge))
		:effect (and (at-pose ?endedge) (not (at-pose ?initedge))
						(increase (time-cost) 1)
						(increase (place-qual) 0)
						))

	(:action Release
		:parameters (?cloth - garment ?edge - grasp)
		:precondition (and (PP) (not (Lifted)) (at-garment ?cloth) (at-pose ?edge) )
		:effect (and (not (PP))
						(increase (time-cost) 1)
						(increase (place-qual) 0)
						))

	(:action Lift
		:parameters (?cloth - garment ?edge - grasp)
		:precondition (and (PP) (not (Placed)) (at-garment ?cloth) (at-pose ?edge))
		:effect (and (Lifted)
						(increase (time-cost) 1)
						(increase (place-qual) 0)
						))

	(:action Lift2
		:precondition (and (PP) (not (Placed)))
		:effect (and (Lifted2)
						(increase (time-cost) 10)
						(increase (place-qual) 0)
						))

	(:action Lift3
		:precondition (and (PP) (Lifted2))
		:effect (and (Lifted)
						(increase (time-cost) 10)
						(increase (place-qual) 0)
						))


;; PLACE ACTIONS  - depends first on specified costs (and later on perceived deformation

	(:action PlaceVert
		:parameters (?cloth - garment ?edge - grasp)
		:precondition (and (Lifted) (PP) (at-pose ?edge))
		:effect (and (Placed) (not (Lifted))
				 (increase (time-cost) 10)
				 (increase (place-qual) (place-succ ?cloth ?edge placevert))
				 ))

	(:action PlaceDiag
		:parameters (?cloth - garment ?edge - grasp)
		:precondition (and (Lifted) (PP) (at-pose ?edge))
		:effect (and (Placed) (not (Lifted))
					(increase (time-cost) 1)
					(increase (place-qual) (place-succ ?cloth ?edge placediag))
					))

	(:action PlaceRot
		:parameters (?cloth - garment ?edge - grasp)
		:precondition (and (Lifted) (PP) (at-pose ?edge))
		:effect (and (Placed) (not (Lifted))
					(increase (time-cost) 1)
					(increase (place-qual) (place-succ ?cloth ?edge placerot))
					))

)
