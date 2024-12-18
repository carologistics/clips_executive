(defrule create-rl-env-state-service-init
" Create a service allowing clients to recieve an observation of the current state of the environment. "
    (not (cx-rl-interfaces-create-rl-env-state-service (name "create_rl_env_state")))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-create-rl-env-state-create-service "create_rl_env_state")
    (printout info "Created service for /create_rl_env_state" crlf)
)

(deffunction cx-rl-interfaces-create-rl-env-state-service-callback (?service-name ?request ?response)
    (printout info "Generating environment state string" crlf)
    (bind ?state-string "{")
    (do-for-all-facts ((?df domain-fact))
            TRUE
        (bind ?fact-string (str-cat "\"" ?df:name "(" (create-slot-value-string ?df:param-values) ")\","))
        (bind ?state-string (str-cat ?state-string ?fact-string))
    )
    (bind ?state-string (str-cat (sub-string 1 (- (str-length ?state-string) 1) ?state-string) "}"))

    (cx-rl-interfaces-create-rl-env-state-response-set-field ?response "state" ?state-string)
)