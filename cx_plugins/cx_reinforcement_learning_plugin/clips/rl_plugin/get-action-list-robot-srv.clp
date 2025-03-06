(defrule get-action-list-robot-service-init
" Create a service allowing clients to recieve a list of all executable actions for a given robot. "
    (not (cx-rl-interfaces-get-action-list-robot-service (name "get_action_list_executable_for_robot")))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-get-action-list-robot-create-service "get_action_list_executable_for_robot")
    (printout info "Created service for /get_action_list_executable_for_robot" crlf)
)

(deffunction cx-rl-interfaces-get-action-list-robot-service-callback (?service-name ?request ?response)
    (bind ?robot (cx-rl-interfaces-get-action-list-robot-request-get-field ?request "robot"))
    (printout info "Generating list of all executable actions for " ?robot crlf)
    (bind ?action-list (create$))
    (do-for-all-facts ((?action rl-action))
            (and    (eq ?action:is-selected FALSE)
                    (eq ?action:assigned-to (sym-cat ?robot)))
        (bind ?action-string (str-cat ?action:id "|" ?action:name))
        (printout info "Executable action: " ?action-string crlf)
        (bind ?action-list (insert$ ?action-list 1 ?action-string))
    )
    (cx-rl-interfaces-get-action-list-robot-response-set-field ?response "actions" ?action-list)
)