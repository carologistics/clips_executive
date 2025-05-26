(deffunction plan-assert-action (?name ?robot $?param-values)
" Assert an action with a unique id."
	(bind ?id-sym (gensym*))
	(bind ?id-str (sub-string 4 (str-length ?id-sym) (str-cat ?id-sym)))
	(assert (plan-action (id (string-to-field ?id-str)) (action-name ?name) (robot (str-cat ?robot)) (param-values $?param-values)))
)

(deffunction plan-assert-sequential (?plan-name ?goal-id $?action-tuples)
	(bind ?plan-id (sym-cat ?plan-name (gensym*)))
	(assert (plan (id ?plan-id) (goal-id ?goal-id) (type SEQUENTIAL)))
	(bind ?actions (create$))
	; action tuples might contain FALSE some cases, filter them out
	(foreach ?pa $?action-tuples
		(if ?pa then
			(bind ?actions (append$ ?actions ?pa))
		)
	)
	(foreach ?pa $?actions
		(modify ?pa (id ?pa-index) (plan-id ?plan-id) (goal-id ?goal-id))
	)
)

(defrule goal-expander-stack
    (declare (salience ?*SALIENCE-HIGH*))
    ?g <-   (goal   (id ?goalid)
                    (class STACK)
                    (mode SELECTED)
                    (params upper ?upper lower ?lower)
            )
    (rl-action (id ?goalid) (is-selected TRUE) (assigned-to ?robot&~nil))
    (domain-fact (name can-hold) (param-values ?robot))
    (domain-fact (name clear) (param-values ?upper))
    (domain-fact (name clear) (param-values ?lower))
    (domain-fact (name on-table) (param-values ?upper))
    =>
    (plan-assert-sequential (sym-cat STACK-PLAN (gensym*)) ?goalid
        (plan-assert-action pickup ?robot ?robot ?upper)
        (plan-assert-action stack ?robot ?robot ?upper ?lower)
    )
    (modify ?g (mode EXPANDED))
)

(defrule goal-reasoner-select 
	(rl-action (id ?aid) (is-selected TRUE))
	?g <- (goal (id ?aid) (mode FORMULATED))
	=>
	(modify ?g (mode SELECTED))
)

(defrule goal-reasoner-commit
	?g <- (goal (id ?goal-id) (mode EXPANDED))
    (plan (id ?plan-id) (goal-id ?goal-id))
	=>
	(modify ?g (mode COMMITTED) (committed-to ?plan-id))
)

(defrule goal-reasoner-dispatch
	?g <- (goal (mode COMMITTED))
	=>
	(modify ?g (mode DISPATCHED))
)

(defrule goal-reasoner-completed
	?g <- (goal (id ?goal-id) (mode FINISHED) (outcome ?outcome&~UNKNOWN))
	?a <- (rl-action (id ?goal-id) (is-selected TRUE))
	=>
	(modify ?a (is-finished TRUE))
	(printout t "Goal '" ?goal-id "' has been completed, cleaning up" crlf)
	(delayed-do-for-all-facts ((?p plan)) (eq ?p:goal-id ?goal-id)
		(delayed-do-for-all-facts ((?a plan-action)) (eq ?a:plan-id ?p:id)
			(retract ?a)
		)
		(retract ?p)
	)
	(retract ?g)
)

(defrule skill-action-final
	?pa <- (plan-action (goal-id ?goal-id) (plan-id ?plan-id) (id ?id) (executor ?exec)
	                    (action-name ?action-name) (state PENDING) (robot ?robot))
	=>
	(printout t "Execution of " ?action-name " completed successfully" crlf)
	(modify ?pa (state EXECUTION-SUCCEEDED))
)