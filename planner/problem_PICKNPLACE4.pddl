(define (problem PICKNPLACE2)
	(:domain PICKNPLACE2)
	(:objects towel napkin waffle - garment
						placevert placediag placerot - placing
						multedges singledge - grasp)

(:init (GrWS) (RotWS) (not (PP)) (MultEdgesUp) (at-garment towel) (at-pose multedges) ;;(type-garment towel) (edge-pose multedges)
	(= (time-cost) 0)
	(= (place-qual) 0)
	;; Define placing qualities according to garment, grasp and placing
	(= (place-succ towel multedges placediag) 200)
	(= (place-succ towel multedges placevert) 20)
	(= (place-succ towel singledge placediag) 2000)
	(= (place-succ towel singledge placevert) 10)
	)

(:goal (and (Placed)))

;;(:metric minimize (time-cost))
(:metric minimize (place-qual))
;;(:metric minimize (+ (* 1 (time-cost)) (* 100000000 (place-qual))))
)
