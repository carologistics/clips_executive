(define (domain blocksworld)
(:requirements :strips :typing)

(:types
	block - object
    robot - object
)

(:predicates 
	(clear ?a - block)
    (on-table ?a - block)
    (can-hold ?r - robot)
    (holding ?r - robot ?a - block)
    (on ?a - block ?b - block)
)

(:action pickup
  :parameters (?r - robot ?b - block)
  :precondition (and (clear ?b) (on-table ?b) (can-hold ?r))
  :effect (and (holding ?r ?b) (not (clear ?b)) (not (on-table ?b)) 
               (not (can-hold ?r))))

(:action putdown
  :parameters  (?r - robot ?b - block)
  :precondition (and (holding ?r ?b))
  :effect (and (clear ?b) (can-hold ?r) (on-table ?b) 
               (not (holding ?r ?b))))

(:action stack
  :parameters  (?r - robot ?u - block ?l - block)
  :precondition (and  (clear ?l) (holding ?r ?u))
  :effect (and (can-hold ?r) (clear ?u) (on ?u ?l)
               (not (clear ?l)) (not (holding ?r ?u))))

(:action unstack
  :parameters  (?r - robot ?u - block ?l - block)
  :precondition (and (on ?u ?l) (clear ?u) (can-hold ?r))
  :effect (and (holding ?r ?u) (clear ?l)
               (not (on ?u ?l)) (not (clear ?u)) (not (can-hold ?r))))
)
                    