(deftemplate reset-game
 	(slot stage (type SYMBOL))
)

(deftemplate training-counter
  (slot num (type INTEGER))
)


(defglobal
  ?*SALIENCE-RESET-GAME-HIGH* = ?*SALIENCE-HIGH*
  ?*SALIENCE-RESET-GAME-MIDDLE* = 800
  ?*SALIENCE-RESET-GAME-LOW* = 300
  ?*RESET-GAME-TIMER* = 1.0
)


(defrule assert-training-counter
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  (not (training-counter (num ?n)))
	;(no-reset-on-training-start)
	=>
  (assert (training-counter (num 0)))
)

(defrule reset-game-stage-zero
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  ?r <- (reset-game (stage STAGE-0))
  (rl-mode (mode ?mode))
  (cx-rl-interfaces-reset-cx-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  =>
  (reset)
  (load-facts reset-save)
  (retract ?r)
  (assert (cx-rl-interfaces-reset-cx-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr)))
  (assert (rl-mode (mode ?mode)))
  (assert (reset-game-finished))
)