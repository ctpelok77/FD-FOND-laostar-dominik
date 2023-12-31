(define (domain faults)
 (:types operation tfault)
 (:constants  f1 f2 f3 f4 f5 f6 f7 - tfault
              o1 o2 o3 o4 o5 o6 o7 o8 o9 - operation)
 (:predicates
   (not-completed ?o - operation)
   (completed ?o - operation)
   (fault ?f - tfault)
   (not-fault ?f - tfault)
   (faulted-op ?o - operation ?f - tfault)
   (last-fault ?f - tfault)
   (not-last-fault ?f - tfault)
   (made)
  )

 (:action perform-operation-1-fault
  :parameters (?o - operation)
  :precondition (and  (not-fault f1) (not-fault f2) (not-fault f3) (not-fault f4) (not-fault f5) (not-fault f6) (not-fault f7) (not-completed ?o))
  :effect (and (completed ?o) (not (not-completed ?o))
               (oneof (and) (and (fault f1) (not (not-fault f1))
                                 (faulted-op ?o f1) (last-fault f1) (not (not-last-fault f1)))))
 )
 (:action perform-operation-2-fault
  :parameters (?o - operation)
  :precondition (and  (fault f1) (not-fault f2) (not-fault f3) (not-fault f4) (not-fault f5) (not-fault f6) (not-fault f7) (not-completed ?o))
  :effect (and (completed ?o) (not (not-completed ?o))
               (oneof (and) (and (fault f2) (not (not-fault f2))
                                 (faulted-op ?o f2) (last-fault f2) (not (not-last-fault f2)))))
 )
 (:action perform-operation-3-fault
  :parameters (?o - operation)
  :precondition (and  (fault f1) (fault f2) (not-fault f3) (not-fault f4) (not-fault f5) (not-fault f6) (not-fault f7) (not-completed ?o))
  :effect (and (completed ?o) (not (not-completed ?o))
               (oneof (and) (and (fault f3) (not (not-fault f3))
                                 (faulted-op ?o f3) (last-fault f3) (not (not-last-fault f3)))))
 )
 (:action perform-operation-4-fault
  :parameters (?o - operation)
  :precondition (and  (fault f1) (fault f2) (fault f3) (not-fault f4) (not-fault f5) (not-fault f6) (not-fault f7) (not-completed ?o))
  :effect (and (completed ?o) (not (not-completed ?o))
               (oneof (and) (and (fault f4) (not (not-fault f4))
                                 (faulted-op ?o f4) (last-fault f4) (not (not-last-fault f4)))))
 )
 (:action perform-operation-5-fault
  :parameters (?o - operation)
  :precondition (and  (fault f1) (fault f2) (fault f3) (fault f4) (not-fault f5) (not-fault f6) (not-fault f7) (not-completed ?o))
  :effect (and (completed ?o) (not (not-completed ?o))
               (oneof (and) (and (fault f5) (not (not-fault f5))
                                 (faulted-op ?o f5) (last-fault f5) (not (not-last-fault f5)))))
 )
 (:action perform-operation-6-fault
  :parameters (?o - operation)
  :precondition (and  (fault f1) (fault f2) (fault f3) (fault f4) (fault f5) (not-fault f6) (not-fault f7) (not-completed ?o))
  :effect (and (completed ?o) (not (not-completed ?o))
               (oneof (and) (and (fault f6) (not (not-fault f6))
                                 (faulted-op ?o f6) (last-fault f6) (not (not-last-fault f6)))))
 )
 (:action perform-operation-7-fault
  :parameters (?o - operation)
  :precondition (and  (fault f1) (fault f2) (fault f3) (fault f4) (fault f5) (fault f6) (not-fault f7) (not-completed ?o))
  :effect (and (completed ?o) (not (not-completed ?o))
               (oneof (and) (and (fault f7) (not (not-fault f7))
                                 (faulted-op ?o f7) (last-fault f7) (not (not-last-fault f7)))))
 )
 (:action repair-fault-1
  :parameters (?o - operation)
  :precondition (and (faulted-op ?o f1) (last-fault f1))
  :effect (and (not (faulted-op ?o f1))
               (not-completed ?o) (not (completed ?o))
               (not-last-fault f1) (not (last-fault f1)) (not-fault f1)
          )
  )
 (:action repair-fault-2
  :parameters (?o - operation)
  :precondition (and (faulted-op ?o f2) (last-fault f2))
  :effect (and (not (faulted-op ?o f2))
               (not-completed ?o) (not (completed ?o))
               (last-fault f1) (not (not-last-fault f1))
               (not-last-fault f2) (not (last-fault f2)) (not-fault f2)
          )
  )
 (:action repair-fault-3
  :parameters (?o - operation)
  :precondition (and (faulted-op ?o f3) (last-fault f3))
  :effect (and (not (faulted-op ?o f3))
               (not-completed ?o) (not (completed ?o))
               (last-fault f2) (not (not-last-fault f2))
               (not-last-fault f3) (not (last-fault f3)) (not-fault f3)
          )
  )
 (:action repair-fault-4
  :parameters (?o - operation)
  :precondition (and (faulted-op ?o f4) (last-fault f4))
  :effect (and (not (faulted-op ?o f4))
               (not-completed ?o) (not (completed ?o))
               (last-fault f3) (not (not-last-fault f3))
               (not-last-fault f4) (not (last-fault f4)) (not-fault f4)
          )
  )
 (:action repair-fault-5
  :parameters (?o - operation)
  :precondition (and (faulted-op ?o f5) (last-fault f5))
  :effect (and (not (faulted-op ?o f5))
               (not-completed ?o) (not (completed ?o))
               (last-fault f4) (not (not-last-fault f4))
               (not-last-fault f5) (not (last-fault f5)) (not-fault f5)
          )
  )
 (:action repair-fault-6
  :parameters (?o - operation)
  :precondition (and (faulted-op ?o f6) (last-fault f6))
  :effect (and (not (faulted-op ?o f6))
               (not-completed ?o) (not (completed ?o))
               (last-fault f5) (not (not-last-fault f5))
               (not-last-fault f6) (not (last-fault f6)) (not-fault f6)
          )
  )
 (:action repair-fault-7
  :parameters (?o - operation)
  :precondition (and (faulted-op ?o f7) (last-fault f7))
  :effect (and (not (faulted-op ?o f7))
               (not-completed ?o) (not (completed ?o))
               (last-fault f6) (not (not-last-fault f6))
               (not-last-fault f7) (not (last-fault f7)) (not-fault f7)
          )
  )
 (:action finish
  :precondition (and  (completed o1) (completed o2) (completed o3) (completed o4) (completed o5) (completed o6) (completed o7) (completed o8) (completed o9) (not-last-fault f7))
  :effect (made)
 )
)

