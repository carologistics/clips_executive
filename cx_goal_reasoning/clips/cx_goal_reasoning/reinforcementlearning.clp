(defglobal
  ?*SALIENCE-RL-SELECTION* = ?*SALIENCE-HIGH*
)

(deftemplate rl-goal-selection
	(slot next-goal-id (type SYMBOL))
)

(deftemplate rl-finished-goal
	(slot goal-id (type SYMBOL));
	(slot outcome (type SYMBOL));
	(slot reward (type INTEGER));
)


;(deffunction remove-robot-assignment-from-goal-meta (?goal)
;  (if (not (do-for-fact ((?f goal-meta))
;      (eq ?f:goal-id (fact-slot-value ?goal id))
;      (modify ?f ( nil))
;      ))
;   then
;    (printout t "Cannot find a goal meta fact for the goal " ?goal crlf)
;  )
;)

(defrule rl-clips-goal-selection
  (declare (salience ?*SALIENCE-RL-SELECTION*))
	?r <- (rl-goal-selection (next-goal-id ?a))
	?next-goal <- (goal (id ?a) (mode ?m&FORMULATED) (assigned-to ?robot))
	=>
	(printout t crlf "in RL Plugin added fact: " ?r " with next action " ?a crlf )
	(printout t crlf "goal: " ?next-goal "with in mode: "?m crlf crlf)
	
	;(retract ?r)
  (modify ?next-goal (mode SELECTED))

  (delayed-do-for-all-facts ((?g goal))
		(and (eq ?g:is-executable TRUE) (neq ?g:class SEND-BEACON))
		(modify ?g (is-executable FALSE))
	)
  ; if it is actually a robot, remove all other assignments and the waiting status
	(if (and (neq ?robot central) (neq ?robot nil))
		then
		(delayed-do-for-all-facts ((?g goal))
			(and (eq ?g:mode FORMULATED) (not (eq ?g:type MAINTAIN)))
			(modify ?g (assigned-to nil))
		)
		(do-for-fact ((?waiting wm-fact))
			(and (wm-key-prefix ?waiting:key (create$ central agent robot-waiting))
			     (eq (wm-key-arg ?waiting:key r) ?robot))
			(retract ?waiting)
		)
	)
)


(defrule rl-selected-goal-finished
  (declare (salience ?*SALIENCE-RL-SELECTION*))
	?r <- (rl-goal-selection (next-goal-id ?goal-id))
	(goal (id ?goal-id) (class ?goal-class) (mode ?mode&FINISHED|EVALUATED) (outcome ?outcome) (points ?points))
	=>
	(printout t crlf "Goal: " ?goal-id " is " ?mode crlf )

    (if (eq ?outcome COMPLETED) 
    then
        (bind ?reward ?points)
    else
        (bind ?reward 0)
    )
 
    (assert (rl-finished-goal (goal-id ?goal-id) (outcome ?outcome) (reward ?reward)))
	(retract ?r)
)


(defrule delete-all-rl-selections
  (declare (salience ?*SALIENCE-FIRST*))
  (rl-delete-selections)
  ?r <-(rl-goal-selection (next-goal-id ?goal-id))
  =>
  (assert (rl-finished-goal (goal-id ?goal-id) (outcome RESET) (reward 0)))
  (retract ?r)
)

(defrule finished-deleting-rl-selections
  (declare (salience ?*SALIENCE-FIRST*))
  ?r <- (rl-delete-selections)
  (not (rl-goal-selection))
  =>
  (retract ?r)
)