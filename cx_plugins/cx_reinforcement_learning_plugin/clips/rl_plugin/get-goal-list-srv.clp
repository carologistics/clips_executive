(defrule get-goal-list-service-init
" Create a service allowing clients to recieve a list of all executable goals. "
    (not (cx-rl-interfaces-get-goal-list-service (name "get_goal_list_executable")))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-get-goal-list-create-service "get_goal_list_executable")
    (printout info "Created service for /get_goal_list_executable" crlf)
)

(deffunction cx-rl-interfaces-get-goal-list-service-callback (?service-name ?request ?response)
    (printout info "Generating list of all executable goals" crlf)
    (bind ?goal-list (create$))
    (do-for-all-facts ((?goal goal))
            (and    (eq ?goal:mode FORMULATED)
                    (eq ?goal:is-executable TRUE))
        (bind ?goal-string (str-cat ?goal:class "#" ?goal:id "#" (create-goal-param-string ?goal:params)))
        (printout info "Executable goal: " ?goal-string crlf)
        (bind ?goal-list (insert$ ?goal-list 1 ?goal-string))
    )
    (cx-rl-interfaces-get-goal-list-response-set-field ?response "goals" ?goal-list)
)

