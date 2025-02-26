(defglobal
  
  ?*SALIENCE-ROBOT-INIT* = 501
  ?*SALIENCE-ACTION-EXECUTABLE-CHECK* = 500
  ?*SALIENCE-RL-EPISODE-END* = 499
  ?*SALIENCE-RL-SELECTION* = 498
)

(deftemplate rl-action
  (slot actionid (type SYMBOL))
  (slot class (type SYMBOL))
  (slot mode  (type SYMBOL)
              (allowed-values FORMULATED SELECTED FINISHED)
              (default FORMULATED))
  (multislot params)
  (slot is-executable (type SYMBOL)
                      (allowed-values TRUE FALSE)
                      (default FALSE))
  (slot assigned-to (type SYMBOL) 
                    (default nil))
  (slot outcome (type SYMBOL) 
                (default UNKNOWN))
  (slot points  (type INTEGER) 
                (default 0))
)

(deftemplate rl-action-selection
	(slot uuid (type STRING))
  (slot actionid (type SYMBOL))
  (slot outcome (type SYMBOL) 
                (default UNKNOWN))
  (slot reward  (type INTEGER)
                (default 0))
  (slot done  (type SYMBOL)
              (allowed-values TRUE FALSE)
              (default FALSE))
)

(deftemplate rl-action-selection-exec
  (slot actionid (type SYMBOL))
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

(deftemplate rl-action-selection-requested)

(deffunction rl-action-selected-update-actions ()
  (delayed-do-for-all-facts ((?a rl-action))
		(eq ?a:is-executable TRUE)
		(modify ?a (is-executable FALSE))
	)
)

(deffunction rl-action-selected-update-robots (?robot)
	(if (neq ?robot nil) then
		(delayed-do-for-all-facts ((?a rl-action))
			(eq ?a:mode FORMULATED)
			(modify ?a (assigned-to nil))
		)
	)
)

(defrule rl-action-select
  (declare (salience ?*SALIENCE-RL-SELECTION*))
  (rl-mode (mode TRAINING|EVALUATION))
	?r <- (rl-action-selection (actionid ?a))
	?next-action <- (rl-action (actionid ?a) (mode FORMULATED) (assigned-to ?robot))
	=>
	(printout info crlf "CXRL: Selected action " ?a  "for robot " ?robot crlf )
	
  (modify ?next-action (mode SELECTED))
  (rl-action-selected-update-actions)
  (rl-action-selected-update-robots ?robot)
  
)

(defrule rl-action-finished
  (declare (salience ?*SALIENCE-RL-SELECTION*))
  (rl-mode (mode TRAINING|EVALUATION))
	?r <- (rl-action-selection (actionid ?actionid))
	?a <- (rl-action (actionid ?actionid) (mode FINISHED) (outcome ?outcome) (points ?points))
	=>
	(printout info crlf "CXRL: Action " ?actionid " has finished" crlf )

  (if (eq ?outcome COMPLETED) then
    (bind ?reward ?points)
  else
    (bind ?reward 0)
  )
 
  (modify ?r (actionid ?actionid) (outcome ?outcome) (reward ?reward) (done FALSE))
  (retract ?a)
)

(defrule rl-action-finished-episode-end
  (declare (salience (+ ?*SALIENCE-RL-SELECTION* 1)))
  (rl-mode (mode TRAINING|EVALUATION))
	?r <- (rl-action-selection (actionid ?actionid))
	?a <- (rl-action (actionid ?actionid) (mode FINISHED) (outcome ?outcome) (points ?points))
  ?e <- (rl-episode-end (success ?success))
	=>
	(printout info crlf "CXRL: Action " ?actionid " has finished, end of episode"crlf )

  (if (eq ?outcome COMPLETED) then
    (bind ?reward ?points)
  else
    (bind ?reward 0)
  )

  (if (eq ?success FALSE) then
    (bind ?reward ?*POINTS-EPISODE-END-FAILURE*)
    (bind ?outcome EPISODE-END-FAILURE)
  else
    (bind ?reward (+ ?reward ?*POINTS-EPISODE-END-SUCCESS*))
  )
 
  (modify ?r (actionid ?actionid) (outcome ?outcome) (reward ?reward) (done TRUE))
  (retract ?e)
  (retract ?a)
)

(defrule domain-game-finished-failure
  (declare (salience ?*SALIENCE-RL-EPISODE-END*))
  (rl-mode (mode TRAINING|EVALUATION))
  (rl-action (assigned-to ?robot&~nil))
  (not (rl-action (assigned-to ?robot) (is-executable TRUE)))
  (not (rl-action (mode SELECTED)))
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
  (not (rl-action-selection-requested))
  (rl-action (mode FORMULATED) (assigned-to ?robot&~nil) (is-executable TRUE))
  (not (rl-episode-end))
  =>
  (assert (rl-action-selection-requested))
)

(defrule rl-action-select-execution
  (declare (salience ?*SALIENCE-RL-SELECTION*))
  (rl-mode (mode EXECUTION))
	?r <- (rl-action-selection-exec (actionid ?actionid))
  ?re <- (rl-action-selection-requested)
	?next-action <- (rl-action (actionid ?actionid) (mode FORMULATED) (assigned-to ?robot))
	=>
	(printout info crlf "CXRL: Selected action " ?actionid  "for robot " ?robot crlf )
	
	(retract ?re)
  (retract ?r)
  (modify ?next-action (mode SELECTED))
  (rl-action-selected-update-actions)
  (rl-action-selected-update-robots ?robot)
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

(defrule assign-robot-to-rl-actions
	" Before checking rl-actions for their executability, pick a waiting robot
  that should get a new action assigned to it next. "
  (declare (salience ?*SALIENCE-HIGH*))
  (rl-action (actionid ?id) (mode FORMULATED) (is-executable FALSE) (assigned-to nil))
  (robot-waiting (robot ?robot))
  (not (rl-action (assigned-to ?robot)))
  (not  (and (robot-waiting (robot ?robot2&:(neq ?robot2 ?robot)))
            (rl-action (actionid ?id2) (mode FORMULATED) (assigned-to ?robot2))
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
  (delayed-do-for-all-facts ((?a rl-action))
    (and (eq ?a:is-executable FALSE)
         (eq ?a:mode FORMULATED)
         (eq ?a:assigned-to nil))
    (modify ?a (assigned-to ?longest-waiting-robot))
  )
  (retract ?longest-waiting)
  (assert (robot-waiting (robot ?longest-waiting-robot)))
)

(defrule unassign-robot-from-finished-action
  (declare (salience ?*SALIENCE-HIGH*))
  ?a <- (rl-action (mode FINISHED) (assigned-to ~nil))
  =>
  (modify ?a (assigned-to nil))
)