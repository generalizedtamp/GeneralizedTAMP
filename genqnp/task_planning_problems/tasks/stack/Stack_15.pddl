(define (problem Stack_15)
(:domain Stack)
(:objects table obj0 obj1 obj2 obj3 obj4 obj5 obj6 obj7 obj8 obj9 obj10 obj11 obj12 obj13 obj14)
(:init (on-table obj0)
(on-table obj1)
(on obj2 obj0)
(on obj3 obj1)
(on obj4 obj2)
(on-table obj5)
(on obj6 obj3)
(on-table obj7)
(on obj8 obj6)
(on obj9 obj4)
(on-table obj10)
(on obj11 obj7)
(on obj12 obj8)
(on obj13 obj12)
(on obj14 obj5)
(clear obj10)
(clear obj11)
(clear obj13)
(clear obj9)
(clear obj14))
(:goal (and (on-table obj0) (on-table obj1) (on-table obj2) (on-table obj3) (on-table obj4) (on-table obj5) (on-table obj6) (on-table obj7) (on-table obj8) (on-table obj9) (on-table obj10) (on-table obj11) (on-table obj12) (on-table obj13) (on-table obj14)))
)