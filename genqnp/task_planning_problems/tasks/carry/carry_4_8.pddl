(define (problem carry_4_8)
(:domain carry)
(:objects obj0 obj1 obj2 obj3 obj4 obj5 obj6 obj7 slot0 slot1 slot2 slot3 src dest c)
(:init (on obj0 src)
(block obj0)
(on obj1 src)
(block obj1)
(on obj2 src)
(block obj2)
(on obj3 src)
(block obj3)
(on obj4 src)
(block obj4)
(on obj5 src)
(block obj5)
(on obj6 src)
(block obj6)
(on obj7 src)
(block obj7)
(slot slot0)
(slot slot1)
(slot slot2)
(slot slot3)
(src_region src)
(dest_region dest)
(container c)
(region c)
(region dest)
(region src)
(canmove dest src)
(canmove src dest)
(at src)
(at c)
(hfree)
(on c src))
(:goal (and (on obj0 dest) (on obj1 dest) (on obj2 dest) (on obj3 dest) (on obj4 dest) (on obj5 dest) (on obj6 dest) (on obj7 dest)))
)