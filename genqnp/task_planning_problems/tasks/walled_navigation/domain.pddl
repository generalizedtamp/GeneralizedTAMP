(define (domain Navigation)
  (:requirements :strips)
  (:predicates (at ?p1)
  (adjacent ?p1 ?p2)
  (boundary ?p1)
  (reward ?p2)           
  )

  (:action move
         :parameters (?p1 ?p2)
         :precondition (and (not (boundary ?p2)) (adjacent ?p1 ?p2) (at ?p1))
         :effect (and (not (at ?p1)) (at ?p2) (not (reward ?p2)) )
  )
 
)