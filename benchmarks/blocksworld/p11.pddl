(define (problem bw_10_11)
  (:domain blocks-domain)
  (:objects b1 b2 b3 b4 b5 b6 b7 b8 b9 b10 - block)

  (:init (emptyhand)       (clear b1) (clear b3) (clear b6) (clear b7) (clear b8))

  (:goal (and (emptyhand)   (on b3 b1)   (clear b2) (clear b3)))
)
