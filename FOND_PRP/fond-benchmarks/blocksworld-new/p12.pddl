(define (problem bw_12_12)
  (:domain blocks-domain)
  (:objects b1 b2 b3 b4 b5 b6 b7 b8 b9 b10 b11 b12 - block)
  (:init (emptyhand) (on b1 b7) (on b2 b8) (on b3 b11) (on b4 b10) (on-table b5) (on b6 b9) (on b7 b3) (on-table b8) (on b9 b1) (on b10 b2) (on b11 b4) (on-table b12) (clear b5) (clear b6) (clear b12))
  (:goal (and (emptyhand) (on b1 b9) (on-table b2) (on b3 b4) (on b4 b6) (on b5 b1) (on b6 b2) (on b7 b10) (on b8 b5) (on b9 b12) (on-table b10) (on b11 b8) (on-table b12) (clear b3) (clear b7) (clear b11)))
)
