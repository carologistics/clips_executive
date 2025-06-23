(define (problem bw-1)
    (:domain bw)
    
    (:objects A B C - block)
    
    (:init 
        (arm-empty)
        (on-table A)
        (on B A)
        (on C B)
        (clear C)
    )
    
    (:goal 
        (and 
            (on A B)
            (on B C)
        )
    )
)