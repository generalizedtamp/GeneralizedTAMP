(define (problem sort_4_2_2)
(:domain sort)
(:objects or0 or1 ob0 ob1 rred rblue rtable)
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
(table rtable)
(redregion rred)
(blueregion rblue)
(region rtable)
(region rred)
(region rblue)
(hfree))
(:goal (and (on or0 rred) (on or1 rred) (on ob0 rblue) (on ob1 rblue)))
)