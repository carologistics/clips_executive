(defrule get-action-list-service-init
" Create a service allowing clients to recieve a list of all executable actions. "
    (not (cx-rl-interfaces-get-action-list-service (name "get_action_list_executable")))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-get-action-list-create-service "get_action_list_executable")
    (printout info "Created service for /get_action_list_executable" crlf)
)

(deffunction cx-rl-interfaces-get-action-list-service-callback (?service-name ?request ?response)
    (printout info "Generating list of all executable actions" crlf)
    (bind ?action-list (create$))
    (do-for-all-facts ((?action rl-action))
            (eq ?action:is-selected FALSE)
        (bind ?action-string (str-cat ?action:id "|" ?action:name))
        (printout info "Executable action: " ?action-string crlf)
        (bind ?action-list (insert$ ?action-list 1 ?action-string))
    )
    (cx-rl-interfaces-get-action-list-response-set-field ?response "actions" ?action-list)
)

