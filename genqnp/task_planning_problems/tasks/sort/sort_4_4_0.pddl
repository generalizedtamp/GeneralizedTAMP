(define (problem sort_4_4_0)
(:domain sort)
(:objects or0 or1 or2 or3 rred rblue rtable)
(:init (on or0 rtable)
(block or0)
(red or0)
(on or1 rtable)
(block or1)
(red or1)
(on or2 rtable)
(block or2)
(red or2)
(on or3 rtable)
(block or3)
(red or3)
(table rtable)
(redregion rred)
(blueregion rblue)
(region rtable)
(region rred)
(region rblue)
(hfree))
(:goal (and (on or0 rred) (on or1 rred) (on or2 rred) (on or3 rred)))
)