(define (domain Stack)
  (:requirements :strips)
  (:predicates (clear ?x)
               (on-table ?x)
               (holding)
               (on ?x ?y))

  (:action pickup
           :parameters (?ob)
           :precondition (and (clear ?ob) (on-table ?ob))
           :effect (and (holding ) (not (clear ?ob)) (not (on-table ?ob))))

  (:action putdown
           :parameters (?ob)
           :precondition (and (holding ))
           :effect (and (clear ?ob) (on-table ?ob) 
                        (not (holding ))))

  (:action stack
           :parameters (?ob ?underob)
           :precondition (and  (clear ?underob) (holding ))
           :effect (and (clear ?ob) (on ?ob ?underob)
                        (not (clear ?underob)) (not (holding ))))

  (:action unstack
         :parameters (?ob ?underob)
         :precondition (and (on ?ob ?underob) (clear ?ob))
         :effect (and (holding ) (clear ?underob)
                      (not (on ?ob ?underob)) (not (clear ?ob))))
)