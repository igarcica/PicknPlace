(define (problem PICKNPLACE2)
	(:domain PICKNPLACE2)
	(:objects towel napkin waffle - garment
						placevert placediag placerot - placing
						multedges singledge - grasp)

(:init (GrWS) (not (RotWS)) (not (PP)) (MultEdgesUp) (at-garment towel) (at-pose multedges) ;;(type-garment towel) (edge-pose multedges)
	(= (time-cost) 0)
	(= (place-qual) 0)
	;; Define placing qualities according to garment, grasp and placing
	(= (place-succ towel singledge placevert) 1)
	(= (place-succ towel multedges placevert) 10)
	(= (place-succ towel singledge placediag) 10)
	(= (place-succ towel multedges placediag) 10)
	(= (place-succ towel singledge placerot) 10)
	(= (place-succ towel multedges placerot) 10)
	)

(:goal (and (Placed) (not (PP))))

;;(:metric minimize (time-cost))
;;(:metric minimize (place-qual))
(:metric minimize (+ (* 1 (time-cost)) (* 10 (place-qual))))
)
