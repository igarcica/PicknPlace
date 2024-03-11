;; Goes through rotate but it is necessary to have the placings with the defined edge precondition

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
			;;(increase (time-cost) 1)
			(increase (place-qual) 0)
			))

	(:action Approach
		:precondition (and (PP) (not (RotWS)) (not (Lifted)) (not (Placed)) )
		:effect (and (RotWS)
			;;(increase (time-cost) 1)
			(increase (place-qual) 0)
			))

	(:action Rotate
		:parameters (?cloth - garment ?initedge ?endedge - grasp)
		:precondition (and (RotWS) (not (PP)) (not (Placed)) (at-garment ?cloth) (at-pose ?initedge))
		:effect (and (at-pose ?endedge) (not (at-pose ?initedge))
						;;(increase (time-cost) 1)
						(increase (place-qual) 0)
						))

	(:action Release
		:precondition (and (PP) (RotWS) (not (Lifted)))
		:effect (and (not (PP))
						(increase (time-cost) 1)
						(increase (place-qual) 0)
						))

	(:action Lift
		:precondition (and (PP) (not (Placed)))
		:effect (and (Lifted)
						;;(increase (time-cost) 10)
						(increase (place-qual) 0)
						))

	(:action Lift2
		:precondition (and (PP) (not (Placed)))
		:effect (and (Lifted2)
						;(increase (time-cost) 1)
						(increase (place-qual) 0)
						))

	(:action Lift3
		:precondition (and (PP) (Lifted2))
		:effect (and (Lifted)
						;(increase (time-cost) 1)
						(increase (place-qual) 0)
						))


;; PLACE ACTIONS  - depends first on specified costs (and later on perceived deformation

	(:action PlaceVert
		:parameters (?cloth - garment)
		:precondition (and (Lifted) (PP) (at-pose singledge))
		:effect (and (Placed) (not (Lifted))
				 ;;(increase (place-qual) 10)
				 (increase (place-qual) (place-succ ?cloth singledge placevert))
				 ))

	(:action PlaceDiag
		:parameters (?cloth - garment)
		:precondition (and (Lifted) (PP) (at-pose multedges))
		:effect (and (Placed) (not (Lifted))
					;;(increase (place-qual) 1000)
					(increase (place-qual) (place-succ ?cloth multedges placediag))
					))

	(:action PlaceVert2
		:parameters (?cloth - garment)
		:precondition (and (Lifted) (PP) (at-pose multedges))
		:effect (and (Placed) (not (Lifted))
					 ;;(increase (place-qual) 1)
					 (increase (place-qual) (place-succ ?cloth multedges placevert))
					 ))

	(:action PlaceDiag2
		:parameters (?cloth - garment)
		:precondition (and (Lifted) (PP) (at-pose singledge))
		:effect (and (Placed) (not (Lifted))
						;;(increase (place-qual) 1000)
						(increase (place-qual) (place-succ ?cloth singledge placediag))
						))

)
