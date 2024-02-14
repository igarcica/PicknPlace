(define (problem PICKNPLACE3)
	(:domain PICKNPLACE3)
	(:objects towel napkin waffle - garment
						placevert placediag placerot - placing
						multedges singledge - grasp)

(:init (GrWS) (not (RotWS)) (not (PP)) (MultEdgesUp) (at-garment towel multedges) ;;(type-garment towel) (edge-pose multedges)
	;;(= (time-cost) 0)
	;; Define placing qualities according to garment, grasp and placing
	(= (place-succ towel multedges placevert) 70)
	(= (place-succ towel multedges placediag) 90)
	(= (place-succ towel multedges placerot) 90)
	(= (place-succ napkin multedges placevert) 20)
	(= (place-succ napkin multedges placediag) 60)
	(= (place-succ napkin multedges placerot) 80)
	(= (place-succ waffle multedges placevert) 80)
	(= (place-succ waffle multedges placediag) 100)
	(= (place-succ waffle multedges placerot) 100)
	)

(:goal (and (Placed) (at-garment towel multedges)))

;;(:metric minimize (time-cost))
(:metric maximize (place-qual))
)
