(defrule cx-rl-interfaces-exec-action-selection-client-init
"Create a client requesting action selection if there is a robot waiting and at least one action executable"
    (not (cx-rl-interfaces-exec-action-selection-client (service "exec_action_selection")))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-exec-action-selection-create-client "exec_action_selection")
    (printout info "Created client for /exec_action_selection" crlf)
)


(defrule cx-rl-interfaces-send-request-for-action-selection
    (rl-action-selection-requested)
    (not (rl-action-selection))
=> 
    (printout info "action selection demand found" crlf)
    (printout info "Generating observation and list of executable actions" crlf)
    (bind ?state-string "{")

    (do-for-all-facts ((?df domain-fact))
            TRUE
        (bind ?fact-string (str-cat "\"" ?df:name "(" (create-slot-value-string ?df:param-values) ")\","))
        (bind ?state-string (str-cat ?state-string ?fact-string))
    )
    (bind ?state-string (str-cat (sub-string 1 (- (str-length ?state-string) 1) ?state-string) "}"))

    (bind ?action-list (create$))
    (do-for-all-facts ((?action rl-action))
            (eq ?action:is-selected FALSE)
        (bind ?action-string (str-cat ?action:id "|" ?action:name))
        (printout info "Executable action: " ?action-string crlf)
        (bind ?action-list (insert$ ?action-list 1 ?action-string))
    )

    (printout info "Requesting action selection" crlf)
    (bind ?new-req (cx-rl-interfaces-exec-action-selection-request-create))
    (cx-rl-interfaces-exec-action-selection-request-set-field ?new-req "state" ?state-string)
    (cx-rl-interfaces-exec-action-selection-request-set-field ?new-req "actions" ?action-list)
    (cx-rl-interfaces-exec-action-selection-send-request ?new-req "exec_action_selection")
    (cx-rl-interfaces-exec-action-selection-request-destroy ?new-req)
)

(defrule exec-action-selection-response-received
    ?msg-fact <- (cx-rl-interfaces-exec-action-selection-response (service ?service) (msg-ptr ?ptr))
=>
    (bind ?action-id (cx-rl-interfaces-exec-action-selection-response-get-field ?ptr "actionid"))
    (printout green "Received actionid from " ?service ": " ?action-id crlf)
    (assert (rl-action-selection-exec (actionid (sym-cat ?action-id))))
    (cx-rl-interfaces-exec-action-selection-response-destroy ?ptr)
    (retract ?msg-fact)

)
