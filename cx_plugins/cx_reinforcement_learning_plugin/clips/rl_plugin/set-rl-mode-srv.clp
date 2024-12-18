(defrule set-rl-mode-service-init
" Create a service allowing clients to set the rl mode. "
    (not (cx-rl-interfaces-set-rl-mode-service (name "set_rl_mode")))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-set-rl-mode-create-service "set_rl_mode")
    (printout info "Created service for /set_rl_mode" crlf)
)

(deffunction cx-rl-interfaces-set-rl-mode-service-callback (?service-name ?request ?response)
    (bind ?mode (sym-cat (cx-rl-interfaces-set-rl-mode-request-get-field ?request "mode")))
    (printout info "Changing reinforcement learning mode to " ?mode crlf)
    (if (or (eq ?mode TRAINING) (eq ?mode EVALUATION) (eq ?mode EXECUTION)) then 
        (assert (rl-mode (mode ?mode)))
        (cx-rl-interfaces-set-rl-mode-response-set-field ?response "confirmation" (str-cat "Set mode to " ?mode))
    else
        (cx-rl-interfaces-set-rl-mode-response-set-field ?response "confirmation" "Couldn't set mode")
    )
)