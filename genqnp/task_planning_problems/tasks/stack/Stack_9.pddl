(define (problem Stack_9)
(:domain Stack)
(:objects table obj0 obj1 obj2 obj3 obj4 obj5 obj6 obj7 obj8)
(:init (on-table obj0)
(on obj1 obj0)
(on obj2 obj1)
(on-table obj3)
(on obj4 obj2)
(on obj5 obj3)
(on obj6 obj4)
(on-table obj7)
(on obj8 obj6)
(clear obj7)
(clear obj5)
(clear obj8))
(:goal (and (on-table obj0) (on-table obj1) (on-table obj2) (on-table obj3) (on-table obj4) (on-table obj5) (on-table obj6) (on-table obj7) (on-table obj8)))
)