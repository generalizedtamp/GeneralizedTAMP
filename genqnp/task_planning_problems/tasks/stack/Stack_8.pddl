(define (problem Stack_8)
(:domain Stack)
(:objects table obj0 obj1 obj2 obj3 obj4 obj5 obj6 obj7)
(:init (on-table obj0)
(on-table obj1)
(on-table obj2)
(on obj3 obj0)
(on obj4 obj3)
(on obj5 obj1)
(on obj6 obj2)
(on obj7 obj6)
(clear obj4)
(clear obj5)
(clear obj7))
(:goal (and (on-table obj0) (on-table obj1) (on-table obj2) (on-table obj3) (on-table obj4) (on-table obj5) (on-table obj6) (on-table obj7)))
)