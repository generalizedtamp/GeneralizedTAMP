(define (domain FOND_direct_direct)
    (:requirements :non-deterministic)
    (:types variable)
    (:constants n - variable)
    (:predicates
        (zero ?X - variable)
    )

    (:action Putaway
        :precondition (and (holding))
        :effect (and (not (holding)))
    )
    (:action Pick-above-x
        :precondition (and (not (zero n)) (not (holding)))
        :effect (and (oneof (zero n) (not (zero n))) (holding))
    )
    (:action Put-above-x
        :precondition (and (holding))
        :effect (and (not (zero n)) (not (holding)))
    )
    (:action Pick-other
        :precondition (and (not (holding)))
        :effect (and (holding))
    )
)

