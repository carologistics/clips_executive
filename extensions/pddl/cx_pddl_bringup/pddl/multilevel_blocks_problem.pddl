(defrule cx-pddl-clips-agent-poison-plan-before-first-action
  (not (poisoned))
  (pddl-plan (id ?id) (state SUCCESS))
  (not (pddl-action (plan ?id) (state SELECTED)))
  =>
  (do-for-all-facts ((?pa pddl-action)) (eq ?pa:plan ?id)
       (printout green "matched fact " ?pa crlf)
       (modify ?pa (order (+ ?pa:order 1)))
  )
  (assert
    (pddl-action (instance test) (id (gensym*)) (name pick-cube) (params cube_4 parts__0_1) (plan ?id) (order 0) (state IDLE)) 
    (poisoned)
  )
)