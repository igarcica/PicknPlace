;;Removing some predicates (at-garment, etc) and the place-qual cost it considered different rotation actions depending on the location of the edge
;; Considers always MultEdgesDown

(define (domain PICKNPLACE3)
	(:requirements :strips)
	(:types garment grasp placing)
	(:predicates (GrWS) (RotWS) (MultEdgesUp) (MultEdgesLeft) (MultEdgesDown) (MultEdgesRight) (ParallelOrient)
								(PP) (Lifted) (Placed)
								(at-garment ?cloth - garment ?edge - grasp)) ;;(type-garment ?cloth - garment) (edge-pose ?edge - grasp) )
	(:functions (place-qual) (place-succ ?cloth - garment ?edge - grasp ?place - placing))

;; PICK ACTIONS - depends on garment type and initial position

	(:action Grasp
		:parameters (?cloth - garment ?edge - grasp)
		:precondition (and (GrWS) (at-garment ?cloth ?edge)) ;; (edge-pose ?edge))
		:effect (and (PP)
			;;(increase (time-cost) 10)
			))

	(:action Approach
		:precondition (and (PP) (not (MultEdgesDown)) (not (RotWS)) )
		:effect (and (RotWS)
						;;(increase (time-cost) 10)
						))

	(:action Rotate45
		:precondition (and (RotWS) (not (MultEdgesDown)) (MultEdgesRight) (not (PP)))
		:effect (and (MultEdgesDown) (not (MultEdgesRight))
						;;(increase (time-cost) 20)
						))

	(:action Rotate90
		:precondition (and (RotWS) (not (MultEdgesDown)) (MultEdgesUp) (not (PP)))
		:effect (and (MultEdgesDown) (not (MultEdgesUp))
						;;(increase (time-cost) 30)
						))

	(:action Rotate-45
		:precondition (and (RotWS) (not (MultEdgesDown)) (MultEdgesLeft) (not (PP)))
		:effect (and (MultEdgesDown) (not (MultEdgesLeft))
						;;(increase (time-cost) 20)
						))

	(:action Release
		:precondition (and (PP) (RotWS) (not (MultEdgesDown)))
		:effect (and (not (PP))
						;;(increase (time-cost) 10)
						))

	(:action Lift
		:precondition (and (PP) (MultEdgesDown))
		:effect (and (Lifted)
						;;(increase (time-cost) 10)
						))


;; PLACE ACTIONS  - depends first on specified costs (and later on perceived deformation

 (:action PlaceVert
 	 :parameters(?cloth - garment ?edge -grasp ?z - placing)
 	 :precondition (and (Lifted) (PP) (at-garment ?cloth ?edge))
	 :effect (and (Placed) (not (Lifted))
	 				;;(increase (time-cost) 10)
	 				;;(increase (place-qual) (place-succ ?cloth ?edge ?z))
					))

 (:action PlaceDiag
 	 :parameters(?cloth - garment ?edge -grasp ?z - placing)
	 :precondition (and (Lifted) (PP) (at-garment ?cloth ?edge))
	 :effect (and (Placed) (not (Lifted))
	 				;;(increase (time-cost) 10)
					;;(increase (place-qual) (place-succ ?cloth ?edge ?z))
					))

 (:action PlaceRot
 	 :parameters(?cloth - garment ?edge -grasp ?z - placing)
	 :precondition (and (Lifted) (PP) (at-garment ?cloth ?edge))
	 :effect (and (Placed) (not (Lifted))
	 				;;(increase (time-cost) 10)
					;;(increase (place-qual) (place-succ ?cloth ?edge ?z))
					))

)
