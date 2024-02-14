(define (domain pick-and-place)
  (:requirements :strips :typing :fluents)

  ;; Define types
  (:types garment position)

  ;; Define predicates
  (:predicates
    (at-robot ?pos - position)
    (at-garment ?garment - garment ?pos - position)
    (grasped ?garment - garment)
    (placed ?garment - garment ?pos - position)
    (rotated ?garment - garment)
    (lifted ?garment - garment)
  )

  ;; Define fluents
  (:functions
    (time-cost)
    (place-qual)
    (combined-metric)
  )

  ;; Define actions
  (:action approach
    :parameters (?garment - garment ?pos-from - position ?pos-to - position)
    :precondition (and
      (at-robot ?pos-from)
      (at-garment ?garment ?pos-from) )
    :effect (and
      (at-robot ?pos-to)
      (placed ?garment ?pos-to)
      (increase (time-cost) 1)
      (increase (place-qual) 1)
      ;; (= (combined-metric) (/ (time-cost) (+ (place-qual) 1))) ;;; Update combined-metric fluent as the fraction
    )
  )

  ;; Define other actions (grasping, rotation, lifting, placing) similarly


)
