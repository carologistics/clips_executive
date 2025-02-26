(defrule reset-cx-action-server-init
    "Create an action server resetting the clips environment"
    (not (cx-rl-interfaces-reset-cx-server (name "reset_cx")))
    (not (executive-finalize))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-reset-cx-create-server "reset_cx")
    (printout info "Created server for /reset_cx" crlf)
)

(deffunction cx-rl-interfaces-reset-cx-handle-goal-callback (?server ?goal ?uuid)
    (printout blue ?server " callback (goal " ?goal " ; id " ?uuid " )" crlf)
    (return 2)
)

(deffunction cx-rl-interfaces-reset-cx-cancel-goal-callback (?server ?goal ?goal-handle)
    (return 1)
)

(deftemplate reset-cx
    (slot uuid (type STRING))
)

(defrule reset-cx-goal-accepted-start
    (cx-rl-interfaces-reset-cx-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
    (not (reset-cx (uuid ?uuid&:(eq ?uuid (cx-rl-interfaces-reset-cx-server-goal-handle-get-goal-id ?ptr)))))
    (not (reset-game-finished))
=>
    (if (not (cx-rl-interfaces-reset-cx-server-goal-handle-is-canceling ?ptr)) then
        (bind ?uuid (cx-rl-interfaces-reset-cx-server-goal-handle-get-goal-id ?ptr))
        (assert (reset-cx (uuid ?uuid)))
        (assert (reset-game (stage STAGE-0)))
        (assert (abort-all-action-selections))
        (assert (abort-get-free-robot))
    else
        (printout error "Goal immediately canceled" crlf)
    )
)


(defrule reset-cx-finished
    ?ag <- (cx-rl-interfaces-reset-cx-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
    ?rf <- (reset-game-finished)
    (cx-rl-interfaces-reset-cx-server (name "reset_cx"))

=>
    (printout green "reset-game finished" crlf)
    (bind ?result (cx-rl-interfaces-reset-cx-result-create))
    (cx-rl-interfaces-reset-cx-result-set-field ?result "confirmation" "Reset completed")
    (cx-rl-interfaces-reset-cx-server-goal-handle-succeed ?ptr ?result)
    (cx-rl-interfaces-reset-cx-result-destroy ?result)
    (cx-rl-interfaces-reset-cx-server-goal-handle-destroy ?ptr)
    (retract ?rf)
    (retract ?ag)
)  

(defrule reset-cx-server-cleanup
    (executive-finalize)
    (cx-rl-interfaces-reset-cx-server (name ?server))
=>
    (cx-rl-interfaces-reset-cx-destroy-server ?server)
)

(defrule reset-cx-accepted-goal-cleanup
    (executive-finalize)
    ?ag <- (cx-rl-interfaces-reset-cx-accepted-goal (server-goal-handle-ptr ?ptr))
=>
    (cx-rl-interfaces-reset-cx-server-goal-handle-destroy ?ptr)
    (retract ?ag)
)