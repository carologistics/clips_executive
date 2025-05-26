(defrule get-predefined-observables-service-init
" Create a service returning a list of all observable predicates in the clips environment."
    (not (cx-rl-interfaces-get-predefined-observables-service (name "get_predefined_observables")))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-get-predefined-observables-create-service "get_predefined_observables")
    (printout info "Created service for /get_predefined_observables" crlf)
)

(deffunction cx-rl-interfaces-get-predefined-observables-service-callback (?service-name ?request ?response)
    (printout info "Collecting cxrl predefined observables" crlf)
    (bind ?observables (create$))

    (do-for-all-facts ((?po rl-predefined-observable))
            TRUE
        (printout info "predefined observable " ?po:name ?po:params crlf)
        (bind ?observables (insert$ ?observables (+ (length$ ?observables) 1) (str-cat ?po:name "(" (create-slot-value-string ?po:params) ")")))
    )
 
    (cx-rl-interfaces-get-predefined-observables-response-set-field ?response "observables" ?observables)
)