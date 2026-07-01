(define (domain sml)
  (:requirements :strips :typing :equality :negative-preconditions)

  (:types gripper location item - object
          block product - item)

  (:predicates
    (item-at      ?i - item  ?l - location)
    (gripper-free ?r - gripper)
    (initialized  ?p - product)
    (made-from    ?p - product ?b - block)
    (assembly     ?l - location)
  )

  (:action pick
    :parameters (?r - gripper ?i - item ?l - location)
    :precondition (and (item-at ?i ?l) 
                       (gripper-free ?r)
                  )
    :effect (and (not (item-at ?i ?l)) 
                 (not (gripper-free ?r))
            )
    )

  (:action put
    :parameters (?r - gripper ?i - item ?l - location)
    :precondition (and (not (gripper-free ?r))
                       (not (item-at ?i  ?l))
                  )
    :effect (and (item-at ?i ?l) 
                 (gripper-free ?r) 
            )
    
  )

  (:action assemble2
    :parameters (?top - block ?bottom - block ?product - product ?l - location)
    :precondition (and (not (initialized ?product))
                       (made-from ?product ?top)
                       (made-from ?product ?bottom)
                       (item-at   ?top  ?l)
                       (item-at   ?bottom  ?l)
                       (assembly     ?l)
                       (not (= ?top ?bottom))
                  )
    :effect (and
                 (initialized ?product)
                 (item-at ?product ?l)
                 (not (item-at ?top ?l))
                 (not (item-at ?bottom ?l))
            )
  )

  ;(:action assemble3
  ;    :parameters (?block1 - block ?block2 - block ?block3 - block ?product - product ?l - location)
  ;    :precondition (and (item-at ?block1  ?l)
  ;                       (item-at ?block2  ?l)
  ;                       (item-at ?block3  ?l)
  ;                       (made-from ?product ?block1)
  ;                       (made-from ?product ?block2)
  ;                       (made-from ?product ?block3)
  ;                       (not (initialized  ?product))
  ;                       (assembly     ?l)
  ;                       (forall (?b - block) 
  ;                               (imply 
  ;                                    (made-from ?product ?b)
  ;                                    (item-at ?b ?l)
  ;                                  
  ;                               )
  ;                             
  ;                       )
  ;    )
  ;    :effect (and (initialized  ?product)
  ;                 (item-at ?product ?l)
  ;                 (not(item-at ?block1  ?l))
  ;                 (not(item-at ?block2  ?l))
  ;                 (not(item-at ?block3  ?l))
  ;    )
  ;)

)