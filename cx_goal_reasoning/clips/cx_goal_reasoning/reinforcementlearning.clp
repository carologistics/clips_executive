(defglobal
  
  ?*SALIENCE-ROBOT-INIT* = 501
  ?*SALIENCE-GOAL-EXECUTABLE-CHECK* = 500
  ?*SALIENCE-DOMAIN-GAME-FINISHED-FAILURE* = 499
  ?*SALIENCE-RL-SELECTION* = 498
)

(deftemplate rl-goal-selection
	(slot goalid (type SYMBOL))
)

(deftemplate rl-finished-goal
	(slot goalid (type SYMBOL));
	(slot outcome (type SYMBOL));
	(slot reward (type INTEGER));
  (slot done (type SYMBOL))
)

(deftemplate rl-episode-end
  (slot success (type SYMBOL)
                (allowed-values TRUE FALSE)
	              (default TRUE))
)

(deftemplate rl-mode
  (slot mode  (type SYMBOL)
              (allowed-values TRAINING EVALUATION EXECUTION))
)

(deftemplate rl-goal-selection-requested)

(deffunction rl-goal-selected-update-goals ()
  (delayed-do-for-all-facts ((?g goal))
		(eq ?g:is-executable TRUE)
		(modify ?g (is-executable FALSE))
	)
)

(deffunction rl-goal-selected-update-robots (?robot)
	(if (neq ?robot nil) then
		(delayed-do-for-all-facts ((?g goal))
			(and (eq ?g:mode FORMULATED) (not (eq ?g:type MAINTAIN)))
			(modify ?g (assigned-to nil))
		)
	)
)

(defrule rl-clips-goal-selection
  (declare (salience ?*SALIENCE-RL-SELECTION*))
  (rl-mode (mode TRAINING|EVALUATION))
	?r <- (rl-goal-selection (goalid ?a))
	?next-goal <- (goal (id ?a) (mode ?m&FORMULATED) (assigned-to ?robot))
	=>
	(printout info crlf "in RL Plugin added fact: " ?r " with next action " ?a crlf )
	(printout info crlf "goal: " ?next-goal "with in mode: "?m crlf crlf)
	
  (modify ?next-goal (mode SELECTED))
  (rl-goal-selected-update-goals)
  (rl-goal-selected-update-robots ?robot)
  
)


(defrule rl-selected-goal-finished
  (declare (salience ?*SALIENCE-RL-SELECTION*))
  (rl-mode (mode TRAINING|EVALUATION))
	?r <- (rl-goal-selection (goalid ?goalid))
	(goal (id ?goalid) (class ?goal-class) (mode ?mode&FINISHED|EVALUATED) (outcome ?outcome) (points ?points))
	=>
	(printout info crlf "Goal: " ?goalid " is " ?mode crlf )

  (if (eq ?outcome COMPLETED) then
    (bind ?reward ?points)
  else
    (bind ?reward 0)
  )
 
  (assert (rl-finished-goal (goalid ?goalid) (outcome ?outcome) (reward ?reward) (done FALSE)))
	(retract ?r)
)

(defrule rl-selected-goal-finished-episode-end
  (declare (salience (+ ?*SALIENCE-RL-SELECTION* 1)))
  (rl-mode (mode TRAINING|EVALUATION))
	?r <- (rl-goal-selection (goalid ?goalid))
	(goal (id ?goalid) (class ?goal-class) (mode ?mode&FINISHED|EVALUATED) (outcome ?outcome) (points ?points))
  ?e <- (rl-episode-end (success ?success))
	=>
	(printout info crlf "Goal: " ?goalid " is " ?mode crlf )

  (if (eq ?outcome COMPLETED) then
    (bind ?reward ?points)
  else
    (bind ?reward 0)
  )

  (if (eq ?success FALSE) then
    (bind ?reward 0)
    (bind ?outcome EPISODE-END-FAILURE)
  else
    (bind ?reward (+ ?reward 100))
  )
 
  (assert (rl-finished-goal (goalid ?goalid) (outcome ?outcome) (reward ?reward) (done TRUE)))
	(retract ?r)
  (retract ?e)
)

(defrule domain-game-finished-failure
  (declare (salience ?*SALIENCE-DOMAIN-GAME-FINISHED-FAILURE*))
  (rl-mode (mode TRAINING|EVALUATION))
  (goal (assigned-to ?robot&~nil))
  (not (goal (assigned-to ?robot) (is-executable TRUE)))
  (not (goal (mode ~FORMULATED&~FINISHED)))
  (not (rl-episode-end (success ?success)))
  =>
  (assert (rl-episode-end (success FALSE)))
)

(defrule logging-on-episode-end
  (rl-episode-end (success ?success))
  =>
  (if (eq ?success TRUE) then
    (printout info "END OF EPISODE: SUCCESS" crlf)
  else
    (printout info "END OF EPISODE: FAILURE" crlf)
  )
  
)

(defrule rl-execution-demand-selection
  (rl-mode (mode EXECUTION))
  (not (rl-goal-selection-requested))
  (goal (mode FORMULATED) (assigned-to ?robot&~nil) (is-executable TRUE))
  (not (rl-episode-end))
  =>
  (assert (rl-goal-selection-requested))
)

(defrule rl-execution-clips-goal-selection
  (declare (salience ?*SALIENCE-RL-SELECTION*))
  (rl-mode (mode EXECUTION))
	?r <- (rl-goal-selection (goalid ?a))
  ?re <- (rl-goal-selection-requested)
	?next-goal <- (goal (id ?a) (mode ?m&FORMULATED) (assigned-to ?robot))
	=>
	(printout info crlf "in RL Plugin added fact: " ?r " with next action " ?a crlf )
	(printout info crlf "goal: " ?next-goal "with in mode: "?m crlf crlf)
	
	(retract ?re)
  (retract ?r)
  (modify ?next-goal (mode SELECTED))
  (rl-goal-selected-update-goals)
  (rl-goal-selected-update-robots ?robot)
)

;================== ROBOT SELECTION ==================

(deftemplate robot-waiting
  (slot robot (type SYMBOL))
)

(defrule init-robot-waiting
  (declare (salience ?*SALIENCE-ROBOT-INIT*))
  (domain-object (name ?robot) (type robot))
  (not (robot-waiting (robot ?robot)))
  => 
  (assert (robot-waiting (robot ?robot)))
)

(defrule goal-production-assign-robot-to-simple-goals
	" Before checking SIMPLE goals for their executability, pick a waiting robot
  that should get a new goal assigned to it next. "
  (declare (salience ?*SALIENCE-HIGH*))
  (goal (id ?id) (sub-type SIMPLE) (mode FORMULATED) (is-executable FALSE) (assigned-to nil))
  (domain-object (name ?robot) (type robot))
  (not (goal (assigned-to ?robot)))
  (robot-waiting (robot ?robot))
  (not  (and (robot-waiting (robot ?robot2&:(neq ?robot2 ?robot)))
            (goal (id ?id2) (sub-type SIMPLE) (mode FORMULATED) (assigned-to ?robot2))
        )
  )
  =>
  (bind ?longest-waiting 0)
  (bind ?longest-waiting-robot ?robot)
  (delayed-do-for-all-facts ((?waiting robot-waiting))
    TRUE
    (if (or (eq ?longest-waiting 0) (< (fact-index ?waiting) ?longest-waiting))
     then
      (bind ?longest-waiting-robot ?waiting:robot)
      (bind ?longest-waiting (fact-index ?waiting))
    )
  )
  (delayed-do-for-all-facts ((?g goal))
    (and (eq ?g:is-executable FALSE)
         (eq ?g:sub-type SIMPLE) (eq ?g:mode FORMULATED)
         (eq ?g:assigned-to nil))
    (modify ?g (assigned-to ?longest-waiting-robot))
  )
  (retract ?longest-waiting)
  (assert (robot-waiting (robot ?longest-waiting-robot)))
)

(defrule unassign-robot-from-finished-goal
  (declare (salience ?*SALIENCE-HIGH*))
  ?g <- (goal (mode FINISHED) (assigned-to ~nil))
  =>
  (modify ?g (assigned-to nil))
)