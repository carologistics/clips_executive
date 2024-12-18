(defrule goal-selection-action-server-init
    "Create an action server handling the selection of a given goal and collection of its reward after it has finished"
    (not (cx-rl-interfaces-goal-selection-server (name "goal_selection")))
    (not (executive-finalize))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-goal-selection-create-server "goal_selection")
    (printout info "Created server for /goal_selection" crlf)
)

(deffunction cx-rl-interfaces-goal-selection-handle-goal-callback (?server ?goal ?uuid)
    (printout blue ?server " callback (goal " ?goal " ; id " ?uuid " )" crlf)
    (return 2)
)

(deffunction cx-rl-interfaces-goal-selection-cancel-goal-callback (?server ?goal ?goal-handle)
    (return 1)
)

(deftemplate goal-selection
    (slot uuid (type STRING))
    (slot goalid (type SYMBOL))
)

(defrule goal-selection-goal-accepted-start
    (cx-rl-interfaces-goal-selection-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
    (not (goal-selection (uuid ?uuid&:(eq ?uuid (cx-rl-interfaces-goal-selection-server-goal-handle-get-goal-id ?ptr)))))
=>
    (if (not (cx-rl-interfaces-goal-selection-server-goal-handle-is-canceling ?ptr)) then
        (bind ?goal (cx-rl-interfaces-goal-selection-server-goal-handle-get-goal ?ptr))
        (bind ?goalid (cx-rl-interfaces-goal-selection-goal-get-field ?goal "goalid"))
        (bind ?uuid (cx-rl-interfaces-goal-selection-server-goal-handle-get-goal-id ?ptr))
        (assert (goal-selection (uuid ?uuid) (goalid (sym-cat ?goalid))))
        (assert (rl-goal-selection (goalid (sym-cat ?goalid))))
        (bind ?feedback (cx-rl-interfaces-goal-selection-feedback-create))
        (cx-rl-interfaces-goal-selection-feedback-set-field ?feedback "feedback" "Goal selection fact asserted")
        (cx-rl-interfaces-goal-selection-server-goal-handle-publish-feedback ?ptr ?feedback)
        (cx-rl-interfaces-goal-selection-feedback-destroy ?feedback)
    else
        (printout error "Goal immediately canceled" crlf)
    )
)

(defrule goal-selection-abort
    (declare (salience ?*SALIENCE-FIRST*))
    (abort-all-goal-selections)
    ?ag <- (cx-rl-interfaces-goal-selection-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
    ?gs <- (goal-selection (goalid ?goalid) (uuid ?uuid&:(eq ?uuid (cx-rl-interfaces-goal-selection-server-goal-handle-get-goal-id ?ptr))))
=>
    (printout info "ResetCX: Aborting goal " ?goalid crlf)
    (bind ?result (cx-rl-interfaces-goal-selection-result-create))
    (cx-rl-interfaces-goal-selection-result-set-field ?result "goalid" (str-cat ?goalid))
    (cx-rl-interfaces-goal-selection-result-set-field ?result "outcome" "ABORTED")
    (cx-rl-interfaces-goal-selection-result-set-field ?result "reward" 0)
    (cx-rl-interfaces-goal-selection-result-set-field ?result "info" "Aborted")
    (cx-rl-interfaces-goal-selection-server-goal-handle-abort ?ptr ?result)
    (cx-rl-interfaces-goal-selection-result-destroy ?result)
    (cx-rl-interfaces-goal-selection-server-goal-handle-destroy ?ptr)
    (retract ?gs)
    (retract ?ag)
=>

)

(defrule goal-selection-finished
    ?ag <- (cx-rl-interfaces-goal-selection-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
    ?gs <- (goal-selection (goalid ?goalid) (uuid ?uuid&:(eq ?uuid (cx-rl-interfaces-goal-selection-server-goal-handle-get-goal-id ?ptr))))
    ?rf <- (rl-finished-goal (goalid ?goalid) (outcome ?outcome) (reward ?reward) (done ?done))
=>
    (printout green "rl-finished-goal found for goal " ?goalid crlf)
    (if (eq ?done TRUE) then (bind ?info "Done") else (bind ?info ""))
    (bind ?result (cx-rl-interfaces-goal-selection-result-create))
    (cx-rl-interfaces-goal-selection-result-set-field ?result "goalid" (str-cat ?goalid))
    (cx-rl-interfaces-goal-selection-result-set-field ?result "outcome" (str-cat ?outcome))
    (cx-rl-interfaces-goal-selection-result-set-field ?result "reward" ?reward)
    (cx-rl-interfaces-goal-selection-result-set-field ?result "info" ?info)
    (cx-rl-interfaces-goal-selection-server-goal-handle-succeed ?ptr ?result)
    (cx-rl-interfaces-goal-selection-result-destroy ?result)
    (cx-rl-interfaces-goal-selection-server-goal-handle-destroy ?ptr)
    (retract ?rf)
    (retract ?gs)
    (retract ?ag)
)  

(defrule goal-selection-server-cleanup
    (executive-finalize)
    (cx-rl-interfaces-goal-selection-server (name ?server))
=>
    (cx-rl-interfaces-goal-selection-destroy-server ?server)
)

(defrule goal-selection-accepted-goal-cleanup
    (executive-finalize)
    ?ag <- (cx-rl-interfaces-goal-selection-accepted-goal (server-goal-handle-ptr ?ptr))
=>
    (cx-rl-interfaces-goal-selection-server-goal-handle-destroy ?ptr)
    (retract ?ag)
)