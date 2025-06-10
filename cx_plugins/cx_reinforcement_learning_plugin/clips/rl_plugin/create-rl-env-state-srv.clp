(defrule create-rl-env-state-service-init
" Create a service allowing clients to recieve an observation of the current state of the environment. "
    (not (cx-rl-interfaces-create-rl-env-state-service (name "create_rl_env_state")))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-create-rl-env-state-create-service "create_rl_env_state")
    (printout info "Created service for /create_rl_env_state" crlf)
)



(deffunction cx-rl-interfaces-create-rl-env-state-service-callback (?service-name ?request ?response)
    (bind ?state-string (create-observation-string))
    (cx-rl-interfaces-create-rl-env-state-response-set-field ?response "state" ?state-string)
)