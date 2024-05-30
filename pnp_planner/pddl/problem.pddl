(define (problem task)
(:domain picknplacetest)
(:objects
    towel napkin waffle - garment
    multedges singledge - grasp
    placevert placediag placerot - placing
    grws rotws grrotws - workspace
    grasped placed notgrasped lifted - state
    unknown known - corners
    home postgrasp - position
)
(:init
    (garment_obj towel)

    (garment_at grws)

    (at_pose singledge)

    (garment_state notgrasped)

    (corners_pos unknown)

    (robot_at postgrasp)

    (= (time_cost) 0)

)
(:goal (and
    (garment_state placed)
    (garment_at grws)
    (at_pose singledge)
))
(:metric minimize (time_cost))
)
