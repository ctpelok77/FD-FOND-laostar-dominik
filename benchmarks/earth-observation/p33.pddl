(define (problem p33)
  (:domain earth_observation)
  (:objects 
    p11 p12 p13 p14 p15 p16 p17 p21 p22 p23 p24 p25 p26 p27 p31 p32 p33 p34 p35 p36 p37 p41 p42 p43 p44 p45 p46 p47 p51 p52 p53 p54 p55 p56 p57 p61 p62 p63 p64 p65 p66 p67 p71 p72 p73 p74 p75 p76 p77 p81 p82 p83 p84 p85 p86 p87 p91 p92 p93 p94 p95 p96 p97 - patch 
  )
  (:init
    (CONNECTED p11 p22 north-east)
    (CONNECTED p11 p21 east)
    (CONNECTED p12 p23 north-east)
    (CONNECTED p12 p22 east)
    (CONNECTED p12 p21 south-east)
    (CONNECTED p13 p24 north-east)
    (CONNECTED p13 p23 east)
    (CONNECTED p13 p22 south-east)
    (CONNECTED p14 p25 north-east)
    (CONNECTED p14 p24 east)
    (CONNECTED p14 p23 south-east)
    (CONNECTED p15 p26 north-east)
    (CONNECTED p15 p25 east)
    (CONNECTED p15 p24 south-east)
    (CONNECTED p16 p27 north-east)
    (CONNECTED p16 p26 east)
    (CONNECTED p16 p25 south-east)
    (CONNECTED p17 p27 east)
    (CONNECTED p17 p26 south-east)
    (CONNECTED p21 p32 north-east)
    (CONNECTED p21 p31 east)
    (CONNECTED p22 p33 north-east)
    (CONNECTED p22 p32 east)
    (CONNECTED p22 p31 south-east)
    (CONNECTED p23 p34 north-east)
    (CONNECTED p23 p33 east)
    (CONNECTED p23 p32 south-east)
    (CONNECTED p24 p35 north-east)
    (CONNECTED p24 p34 east)
    (CONNECTED p24 p33 south-east)
    (CONNECTED p25 p36 north-east)
    (CONNECTED p25 p35 east)
    (CONNECTED p25 p34 south-east)
    (CONNECTED p26 p37 north-east)
    (CONNECTED p26 p36 east)
    (CONNECTED p26 p35 south-east)
    (CONNECTED p27 p37 east)
    (CONNECTED p27 p36 south-east)
    (CONNECTED p31 p42 north-east)
    (CONNECTED p31 p41 east)
    (CONNECTED p32 p43 north-east)
    (CONNECTED p32 p42 east)
    (CONNECTED p32 p41 south-east)
    (CONNECTED p33 p44 north-east)
    (CONNECTED p33 p43 east)
    (CONNECTED p33 p42 south-east)
    (CONNECTED p34 p45 north-east)
    (CONNECTED p34 p44 east)
    (CONNECTED p34 p43 south-east)
    (CONNECTED p35 p46 north-east)
    (CONNECTED p35 p45 east)
    (CONNECTED p35 p44 south-east)
    (CONNECTED p36 p47 north-east)
    (CONNECTED p36 p46 east)
    (CONNECTED p36 p45 south-east)
    (CONNECTED p37 p47 east)
    (CONNECTED p37 p46 south-east)
    (CONNECTED p41 p52 north-east)
    (CONNECTED p41 p51 east)
    (CONNECTED p42 p53 north-east)
    (CONNECTED p42 p52 east)
    (CONNECTED p42 p51 south-east)
    (CONNECTED p43 p54 north-east)
    (CONNECTED p43 p53 east)
    (CONNECTED p43 p52 south-east)
    (CONNECTED p44 p55 north-east)
    (CONNECTED p44 p54 east)
    (CONNECTED p44 p53 south-east)
    (CONNECTED p45 p56 north-east)
    (CONNECTED p45 p55 east)
    (CONNECTED p45 p54 south-east)
    (CONNECTED p46 p57 north-east)
    (CONNECTED p46 p56 east)
    (CONNECTED p46 p55 south-east)
    (CONNECTED p47 p57 east)
    (CONNECTED p47 p56 south-east)
    (CONNECTED p51 p62 north-east)
    (CONNECTED p51 p61 east)
    (CONNECTED p52 p63 north-east)
    (CONNECTED p52 p62 east)
    (CONNECTED p52 p61 south-east)
    (CONNECTED p53 p64 north-east)
    (CONNECTED p53 p63 east)
    (CONNECTED p53 p62 south-east)
    (CONNECTED p54 p65 north-east)
    (CONNECTED p54 p64 east)
    (CONNECTED p54 p63 south-east)
    (CONNECTED p55 p66 north-east)
    (CONNECTED p55 p65 east)
    (CONNECTED p55 p64 south-east)
    (CONNECTED p56 p67 north-east)
    (CONNECTED p56 p66 east)
    (CONNECTED p56 p65 south-east)
    (CONNECTED p57 p67 east)
    (CONNECTED p57 p66 south-east)
    (CONNECTED p61 p72 north-east)
    (CONNECTED p61 p71 east)
    (CONNECTED p62 p73 north-east)
    (CONNECTED p62 p72 east)
    (CONNECTED p62 p71 south-east)
    (CONNECTED p63 p74 north-east)
    (CONNECTED p63 p73 east)
    (CONNECTED p63 p72 south-east)
    (CONNECTED p64 p75 north-east)
    (CONNECTED p64 p74 east)
    (CONNECTED p64 p73 south-east)
    (CONNECTED p65 p76 north-east)
    (CONNECTED p65 p75 east)
    (CONNECTED p65 p74 south-east)
    (CONNECTED p66 p77 north-east)
    (CONNECTED p66 p76 east)
    (CONNECTED p66 p75 south-east)
    (CONNECTED p67 p77 east)
    (CONNECTED p67 p76 south-east)
    (CONNECTED p71 p82 north-east)
    (CONNECTED p71 p81 east)
    (CONNECTED p72 p83 north-east)
    (CONNECTED p72 p82 east)
    (CONNECTED p72 p81 south-east)
    (CONNECTED p73 p84 north-east)
    (CONNECTED p73 p83 east)
    (CONNECTED p73 p82 south-east)
    (CONNECTED p74 p85 north-east)
    (CONNECTED p74 p84 east)
    (CONNECTED p74 p83 south-east)
    (CONNECTED p75 p86 north-east)
    (CONNECTED p75 p85 east)
    (CONNECTED p75 p84 south-east)
    (CONNECTED p76 p87 north-east)
    (CONNECTED p76 p86 east)
    (CONNECTED p76 p85 south-east)
    (CONNECTED p77 p87 east)
    (CONNECTED p77 p86 south-east)
    (CONNECTED p81 p92 north-east)
    (CONNECTED p81 p91 east)
    (CONNECTED p82 p93 north-east)
    (CONNECTED p82 p92 east)
    (CONNECTED p82 p91 south-east)
    (CONNECTED p83 p94 north-east)
    (CONNECTED p83 p93 east)
    (CONNECTED p83 p92 south-east)
    (CONNECTED p84 p95 north-east)
    (CONNECTED p84 p94 east)
    (CONNECTED p84 p93 south-east)
    (CONNECTED p85 p96 north-east)
    (CONNECTED p85 p95 east)
    (CONNECTED p85 p94 south-east)
    (CONNECTED p86 p97 north-east)
    (CONNECTED p86 p96 east)
    (CONNECTED p86 p95 south-east)
    (CONNECTED p87 p97 east)
    (CONNECTED p87 p96 south-east)
    (CONNECTED p91 p12 north-east)
    (CONNECTED p91 p11 east)
    (CONNECTED p92 p13 north-east)
    (CONNECTED p92 p12 east)
    (CONNECTED p92 p11 south-east)
    (CONNECTED p93 p14 north-east)
    (CONNECTED p93 p13 east)
    (CONNECTED p93 p12 south-east)
    (CONNECTED p94 p15 north-east)
    (CONNECTED p94 p14 east)
    (CONNECTED p94 p13 south-east)
    (CONNECTED p95 p16 north-east)
    (CONNECTED p95 p15 east)
    (CONNECTED p95 p14 south-east)
    (CONNECTED p96 p17 north-east)
    (CONNECTED p96 p16 east)
    (CONNECTED p96 p15 south-east)
    (CONNECTED p97 p17 east)
    (CONNECTED p97 p16 south-east)
    (is-focal-point p14)
    (is-target p14)
    (is-target p15)
    (is-target p21)
    (is-target p23)
    (is-target p27)
    (is-target p32)
    (is-target p36)
    (is-target p41)
    (is-target p43)
    (is-target p44)
    (is-target p47)
    (is-target p55)
    (is-target p63)
    (is-target p71)
    (is-target p72)
    (is-target p73)
    (is-target p74)
    (is-target p75)
    (is-target p77)
    (is-target p81)
    (is-target p82)
    (is-target p83)
    (is-target p86)
    (is-target p91)
    (is-target p94)
    (is-target p96)
  )
  (:goal (and
    (not (is-target p14))
    (not (is-target p15))
    (not (is-target p21))
    (not (is-target p23))
    (not (is-target p27))
    (not (is-target p32))
    (not (is-target p36))
    (not (is-target p41))
    (not (is-target p43))
    (not (is-target p44))
    (not (is-target p47))
    (not (is-target p55))
    (not (is-target p63))
    (not (is-target p71))
    (not (is-target p72))
    (not (is-target p73))
    (not (is-target p74))
    (not (is-target p75))
    (not (is-target p77))
    (not (is-target p81))
    (not (is-target p82))
    (not (is-target p83))
    (not (is-target p86))
    (not (is-target p91))
    (not (is-target p94))
    (not (is-target p96))
  ))
  (:metric minimize (total-cost))
)
