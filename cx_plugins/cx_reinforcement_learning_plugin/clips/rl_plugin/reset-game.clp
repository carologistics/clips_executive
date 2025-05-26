(defglobal
  ?*SALIENCE-RESET-GAME-HIGH* = ?*SALIENCE-HIGH*
  ?*SALIENCE-RESET-GAME-MIDDLE* = 800
  ?*SALIENCE-RESET-GAME-LOW* = 300
  ?*RESET-GAME-TIMER* = 1.0
)

(deftemplate reset-game
 	(slot stage (type SYMBOL))
)

(deffunction delete-rl-actions-after-reset ()
  (delayed-do-for-all-facts ((?r rl-action))
    TRUE
    (retract ?r)
  )
)

(defrule reset-game-stage-zero
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  ?r <- (reset-game (stage STAGE-0))
  (rl-mode (mode ?mode))
  (cx-rl-interfaces-reset-cx-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  =>
  (reset)
  (load-facts reset-save)
  (delete-rl-actions-after-reset)  
  (retract ?r)
  (assert (cx-rl-interfaces-reset-cx-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr)))
  (assert (rl-mode (mode ?mode)))
  (assert (reset-game-finished))
)