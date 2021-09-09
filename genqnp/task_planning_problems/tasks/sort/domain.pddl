(define (domain Sort)
  (:requirements :strips)
  (:predicates (on ?o ?r)
           (holding ?o)
           (red ?o)
           (blue ?o)
           (green ?o)
           (purple ?o)
           (block ?o)
           (redregion ?r)
           (blueregion ?r)
           (orange ?o)
           (table ?r)
           (region ?r)
           (hfree)
    )

  (:action pick
         :parameters (?o ?r)
         :precondition (and (hfree) (on ?o ?r) (block ?o) (region ?r))
         :effect (and (holding ?o) (not (on ?o ?r)) (not (hfree)))
  )

  (:action place
         :parameters (?o ?r)
         :precondition (and (block ?o) (region ?r) (holding ?o))
         :effect (and (on ?o ?r) (not (holding ?o)) (hfree))
  )
)
