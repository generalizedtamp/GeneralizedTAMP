(define (problem sort_6_2_4)
(:domain sort)
(:objects or0 or1 ob0 ob1 ob2 ob3 rred rblue rtable)
(:init (on or0 rtable)
(block or0)
(red or0)
(on or1 rtable)
(block or1)
(red or1)
(on ob0 rtable)
(block ob0)
(blue ob0)
(on ob1 rtable)
(block ob1)
(blue ob1)
(on ob2 rtable)
(block ob2)
(blue ob2)
(on ob3 rtable)
(block ob3)
(blue ob3)
(table rtable)
(redregion rred)
(blueregion rblue)
(region rtable)
(region rred)
(region rblue)
(hfree))
(:goal (and (on or0 rred) (on or1 rred) (on ob0 rblue) (on ob1 rblue) (on ob2 rblue) (on ob3 rblue)))
)