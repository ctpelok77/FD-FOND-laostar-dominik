(define (problem bw_10_15)
  (:domain blocks-domain)
  (:objects b1 b2 b3 b4 b5 b6 b7 b8 b9 b10 - block)
  (:init (emptyhand) (on b1 b7) (on-table b2) (on b3 b9) (on b4 b8) (on b5 b10) (on-table b6) (on-table b7) (on b8 b3) (on b9 b1) (on b10 b2) (clear b4) (clear b5) (clear b6))
  (:goal (and (emptyhand) (on b1 b9) (on b2 b5) (on b3 b2) (on b4 b3) (on b5 b6) (on-table b6) (on b7 b4) (on b8 b7) (on-table b9) (on b10 b1) (clear b8) (clear b10)))
)
