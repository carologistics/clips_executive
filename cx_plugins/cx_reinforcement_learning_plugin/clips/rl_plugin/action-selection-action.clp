(defrule action-selection-action-server-init
    "Create an action server handling the selection of a given action and collection of its reward after it has finished"
    (not (cx-rl-interfaces-action-selection-server (name "action_selection")))
    (not (executive-finalize))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-action-selection-create-server "action_selection")
    (printout info "Created server for /action_selection" crlf)
)

(deffunction cx-rl-interfaces-action-selection-handle-goal-callback (?server ?goal ?uuid)
    (printout blue ?server " callback (goal " ?goal " ; id " ?uuid " )" crlf)
    (return 2)
)

(deffunction cx-rl-interfaces-action-selection-cancel-goal-callback (?server ?goal ?goal-handle)
    (return 1)
)



(defrule action-selection-goal-accepted-start
    (cx-rl-interfaces-action-selection-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
    (not (rl-action-selection (uuid ?uuid&:(eq ?uuid (cx-rl-interfaces-action-selection-server-goal-handle-get-goal-id ?ptr)))))
=>
    (if (not (cx-rl-interfaces-action-selection-server-goal-handle-is-canceling ?ptr)) then
        (bind ?goal (cx-rl-interfaces-action-selection-server-goal-handle-get-goal ?ptr))
        (bind ?actionid (cx-rl-interfaces-action-selection-goal-get-field ?goal "actionid"))
        (bind ?uuid (cx-rl-interfaces-action-selection-server-goal-handle-get-goal-id ?ptr))
        (assert (rl-action-selection (uuid ?uuid) (actionid (sym-cat ?actionid))))
        (bind ?feedback (cx-rl-interfaces-action-selection-feedback-create))
        (cx-rl-interfaces-action-selection-feedback-set-field ?feedback "feedback" "Action selection fact asserted")
        (cx-rl-interfaces-action-selection-server-goal-handle-publish-feedback ?ptr ?feedback)
        (cx-rl-interfaces-action-selection-feedback-destroy ?feedback)
    else
        (printout error "Goal immediately canceled" crlf)
    )
)

(defrule action-selection-abort
    (declare (salience ?*SALIENCE-FIRST*))
    (abort-all-action-selections)
    ?ag <- (cx-rl-interfaces-action-selection-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
    ?as <- (rl-action-selection (actionid ?actionid) (uuid ?uuid&:(eq ?uuid (cx-rl-interfaces-action-selection-server-goal-handle-get-goal-id ?ptr))))
=>
    (printout info "ResetCX: Aborting action " ?actionid crlf)
    (bind ?result (cx-rl-interfaces-action-selection-result-create))
    (cx-rl-interfaces-action-selection-result-set-field ?result "actionid" (str-cat ?actionid))
    (cx-rl-interfaces-action-selection-result-set-field ?result "outcome" "ABORTED")
    (cx-rl-interfaces-action-selection-result-set-field ?result "reward" 0)
    (cx-rl-interfaces-action-selection-result-set-field ?result "info" "Aborted")
    (cx-rl-interfaces-action-selection-server-goal-handle-abort ?ptr ?result)
    (cx-rl-interfaces-action-selection-result-destroy ?result)
    (cx-rl-interfaces-action-selection-server-goal-handle-destroy ?ptr)
    (retract ?as)
    (retract ?ag)
=>

)

(defrule action-selection-finished
    ?ag <- (cx-rl-interfaces-action-selection-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
    ?as <- (rl-action-selection (actionid ?actionid)  (outcome ?outcome&~UNKNOWN) (reward ?reward) 
                                (done ?done) (uuid ?uuid&:(eq ?uuid (cx-rl-interfaces-action-selection-server-goal-handle-get-goal-id ?ptr))))
=>
    (printout green "rl-action finished for action " ?actionid crlf)
    (if (eq ?done TRUE) then (bind ?info "Done") else (bind ?info ""))
    (bind ?result (cx-rl-interfaces-action-selection-result-create))
    (cx-rl-interfaces-action-selection-result-set-field ?result "actionid" (str-cat ?actionid))
    (cx-rl-interfaces-action-selection-result-set-field ?result "outcome" (str-cat ?outcome))
    (cx-rl-interfaces-action-selection-result-set-field ?result "reward" ?reward)
    (cx-rl-interfaces-action-selection-result-set-field ?result "info" ?info)
    (cx-rl-interfaces-action-selection-server-goal-handle-succeed ?ptr ?result)
    (cx-rl-interfaces-action-selection-result-destroy ?result)
    (cx-rl-interfaces-action-selection-server-goal-handle-destroy ?ptr)
    (retract ?as)
    (retract ?ag)
)  

(defrule action-selection-server-cleanup
    (executive-finalize)
    (cx-rl-interfaces-action-selection-server (name ?server))
=>
    (cx-rl-interfaces-action-selection-destroy-server ?server)
)

(defrule action-selection-accepted-goal-cleanup
    (executive-finalize)
    ?ag <- (cx-rl-interfaces-action-selection-accepted-goal (server-goal-handle-ptr ?ptr))
=>
    (cx-rl-interfaces-action-selection-server-goal-handle-destroy ?ptr)
    (retract ?ag)
)