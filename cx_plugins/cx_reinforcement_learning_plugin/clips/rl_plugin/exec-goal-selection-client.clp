(defrule cx-rl-interfaces-exec-goal-selection-client-init
"Create a client requesting goal selection if there is a robot waiting and at least one goal executable"
    (not (cx-rl-interfaces-exec-goal-selection-client (service "exec_goal_selection")))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-exec-goal-selection-create-client "exec_goal_selection")
    (printout info "Created client for /exec_goal_selection" crlf)
)


(defrule cx-rl-interfaces-send-request-for-goal-selection
    (rl-goal-selection-requested)
    (not (rl-goal-selection))
=> 
    (printout info "Goal selection demand found" crlf)
    (printout info "Generating observation and list of executable goals" crlf)
    (bind ?state-string "{")

    (do-for-all-facts ((?df domain-fact))
            TRUE
        (bind ?fact-string (str-cat "\"" ?df:name "(" (create-slot-value-string ?df:param-values) ")\","))
        (bind ?state-string (str-cat ?state-string ?fact-string))
    )
    (bind ?state-string (str-cat (sub-string 1 (- (str-length ?state-string) 1) ?state-string) "}"))

    (bind ?goal-list (create$))
    (do-for-all-facts ((?goal goal))
            (and    (eq ?goal:mode FORMULATED)
                    (eq ?goal:is-executable TRUE))
        (bind ?goal-string (str-cat ?goal:class "#" ?goal:id "#" (create-goal-param-string ?goal:params)))
        (printout info "Executable goal: " ?goal-string crlf)
        (bind ?goal-list (insert$ ?goal-list 1 ?goal-string))
    )

    (printout info "Requesting goal selection" crlf)
    (bind ?new-req (cx-rl-interfaces-exec-goal-selection-request-create))
    (cx-rl-interfaces-exec-goal-selection-request-set-field ?new-req "state" ?state-string)
    (cx-rl-interfaces-exec-goal-selection-request-set-field ?new-req "goals" ?goal-list)
    (cx-rl-interfaces-exec-goal-selection-send-request ?new-req "exec_goal_selection")
    (cx-rl-interfaces-exec-goal-selection-request-destroy ?new-req)
)

(defrule exec-goal-selection-response-received
    ?msg-fact <- (cx-rl-interfaces-exec-goal-selection-response (service ?service) (msg-ptr ?ptr))
=>
    (bind ?goal-id (cx-rl-interfaces-exec-goal-selection-response-get-field ?ptr "goalid"))
    (printout green "Received goalid from " ?service ": " ?goal-id crlf)
    (assert (rl-goal-selection (goalid (sym-cat ?goal-id))))
    (cx-rl-interfaces-exec-goal-selection-response-destroy ?ptr)
    (retract ?msg-fact)

)
