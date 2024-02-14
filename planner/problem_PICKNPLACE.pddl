(define (problem pick-and-place-problem)
  (:domain pick-and-place)

  ;; Define objects
  (:objects
    pos1 pos2 pos3 - position
    shirt - garment
    ;;; Add other objects
  )

	;; Define the initial state
  (:init
    (at-robot pos1)
    (at-garment shirt pos1)
    (= (time-cost) 0)         ; Initial time cost is 0
    (= (place-qual) 0)        ; Initial place quality (you can adjust this)
    (= (combined-metric) 0)   ; Initial combined metric
    ;;; Add other initial conditions
  )

  ;; Define the goal state
  (:goal (and
    (placed shirt pos3)
    (rotated shirt)
    ;;; Add other goal conditions
  ))

  ;; Define the metric
  (:metric minimize (time-cost))
)
