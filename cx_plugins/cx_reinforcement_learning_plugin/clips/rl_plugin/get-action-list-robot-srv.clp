(defrule get-goal-list-robot-service-init
" Create a service allowing clients to recieve a list of all executable goals for a given robot. "
    (not (cx-rl-interfaces-get-goal-list-robot-service (name "get_goal_list_executable_for_robot")))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-get-goal-list-robot-create-service "get_goal_list_executable_for_robot")
    (printout info "Created service for /get_goal_list_executable_for_robot" crlf)
)

(deffunction cx-rl-interfaces-get-goal-list-robot-service-callback (?service-name ?request ?response)
    (bind ?robot (cx-rl-interfaces-get-goal-list-robot-request-get-field ?request "robot"))
    (printout info "Generating list of all executable goals for " ?robot crlf)
    (bind ?goal-list (create$))
    (do-for-all-facts ((?goal goal))
            (and    (eq ?goal:mode FORMULATED)
                    (eq ?goal:is-executable TRUE)
                    (eq ?goal:assigned-to (sym-cat ?robot)))
        (bind ?goal-string (str-cat ?goal:class "#" ?goal:id "#" (create-goal-param-string ?goal:params)))
        (printout info "Executable goal: " ?goal-string crlf)
        (bind ?goal-list (insert$ ?goal-list 1 ?goal-string))
    )
    (cx-rl-interfaces-get-goal-list-robot-response-set-field ?response "goals" ?goal-list)
)