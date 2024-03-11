(define (problem task)
(:domain picknplacetest)
(:objects
    towel napkin waffle - garment
    multedges singledge - grasp
    placevert placediag placerot - placing
    grws rotws grrotws - workspace
    grasped placed notgrasped lifted - state
)
(:init
    (garment-at towel grws)


    (at-pose multedges)

    (garment-state notgrasped)




    (= (time-cost) 0)

)
(:goal (and
    (garment-state placed)
    (at-pose singledge)
))
(:metric minimize (time-cost))
)
