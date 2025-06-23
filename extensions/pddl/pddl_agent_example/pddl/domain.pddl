(define (domain bw)
    (:types
        block - object
    )


    (:predicates
        (on ?x - block ?y - block) ; object ?x is on ?object ?y
        (on-table ?x - block) ; ?x is directly on the table
        (clear ?x - block) ; ?x has nothing on it
        (arm-empty) ; robot isn't holding anything
        (holding ?x - block)) ; robot is holding ?x
    
    (:durative-action pick-up
        :parameters (?ob - block)
        :duration (= ?duration 10)
        :condition
            (and 
                (at start (clear ?ob))
                (at start (on-table ?ob))
                (at start (arm-empty)))
        :effect
            (and 
                (at end (not (on-table ?ob)))
                (at end (not (clear ?ob)))
                (at end (not (arm-empty)))
                (at end (holding ?ob))))

    (:durative-action put-down
        :parameters (?ob - block)
        :duration (= ?duration 10)
        :condition (at start (holding ?ob))
        :effect
            (and 
                (at end (not (holding ?ob)))
                (at end (clear ?ob))
                (at end (arm-empty))
                (at end (on-table ?ob)))
    )

    (:durative-action stack
        :parameters (?ob1 - block ?ob2 - block)
        :duration (= ?duration 10)
        :condition (and (at start (holding ?ob1)) (at start (clear ?ob2)))
        :effect
            (and 
                (at end (not (holding ?ob1)))
                (at end (not (clear ?ob2)))
                (at end (clear ?ob1))
                (at end (arm-empty))
                (at end (on ?ob1 ?ob2)))
    )

    (:durative-action unstack
        :parameters (?ob1 - block ?ob2 - block)
        :duration (= ?duration 10)
        :condition
            (and 
                (at start (on ?ob1 ?ob2))
                (at start (clear ?ob1))
                (at start (arm-empty)))
        :effect
            (and 
                (at end (holding ?ob1))
                (at end (clear ?ob2))
                (at end (not (clear ?ob1)))
                (at end (not (arm-empty)))
                (at end (not (on ?ob1 ?ob2))))
    )
);