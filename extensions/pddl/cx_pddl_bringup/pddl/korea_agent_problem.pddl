(define (problem sml-p1)
  (:domain sml)

  (:objects
    robot1                      - gripper
    stor1-left stor1-right      - location
    stor2-left stor2-right      - location
    stor3-left stor3-right      - location
    cc1 wb1 wb2                 - location
    block3 block4 block2        - block
    block34                     - product
  )

  (:init
    (gripper-free robot1)
    (item-at block3 stor1-right)
    (item-at block4 stor1-right)
    (assembly wb1)
    (made-from block34 block3)
    (made-from block34 block4)
  )

  (:goal (and
    (initialized block34)
    (item-at block34 cc1)
    
    )

  )

)