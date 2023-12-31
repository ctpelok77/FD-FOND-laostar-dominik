(define (problem p01)
  (:domain earth_observation)
  (:objects 
    p11 p12 p13 p21 p22 p23 p31 p32 p33 - patch 
  )
  (:init
    (CONNECTED p11 p22 north-east)
    (CONNECTED p11 p21 east)
    (CONNECTED p12 p23 north-east)
    (CONNECTED p12 p22 east)
    (CONNECTED p12 p21 south-east)
    (CONNECTED p13 p23 east)
    (CONNECTED p13 p22 south-east)
    (CONNECTED p21 p32 north-east)
    (CONNECTED p21 p31 east)
    (CONNECTED p22 p33 north-east)
    (CONNECTED p22 p32 east)
    (CONNECTED p22 p31 south-east)
    (CONNECTED p23 p33 east)
    (CONNECTED p23 p32 south-east)
    (CONNECTED p31 p12 north-east)
    (CONNECTED p31 p11 east)
    (CONNECTED p32 p13 north-east)
    (CONNECTED p32 p12 east)
    (CONNECTED p32 p11 south-east)
    (CONNECTED p33 p13 east)
    (CONNECTED p33 p12 south-east)
    (is-focal-point p12)
    (is-target p11)
    (is-target p13)
    (is-target p21)
    (is-target p23)
    (is-target p31)
    (is-target p33)
  )
  (:goal (and
    (not (is-target p11))
    (not (is-target p13))
    (not (is-target p21))
    (not (is-target p23))
    (not (is-target p31))
    (not (is-target p33))
  ))
  (:metric minimize (total-cost))
)
