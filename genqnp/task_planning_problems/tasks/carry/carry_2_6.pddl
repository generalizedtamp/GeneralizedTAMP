(define (problem carry_2_6)
(:domain carry)
(:objects obj0 obj1 obj2 obj3 obj4 obj5 slot0 slot1 src dest c)
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
(slot slot0)
(slot slot1)
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
(:goal (and (on obj0 dest) (on obj1 dest) (on obj2 dest) (on obj3 dest) (on obj4 dest) (on obj5 dest)))
)