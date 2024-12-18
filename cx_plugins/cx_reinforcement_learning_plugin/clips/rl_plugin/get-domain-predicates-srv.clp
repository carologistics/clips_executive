(defrule get-domain-predicates-service-init
" Create a service returning a list of all domain predicates in the clips environment."
    (not (cx-rl-interfaces-get-domain-predicates-service (name "get_domain_predicates")))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-get-domain-predicates-create-service "get_domain_predicates")
    (printout info "Created service for /get_domain_predicates" crlf)
)

(deffunction cx-rl-interfaces-get-domain-predicates-service-callback (?service-name ?request ?response)
    (printout info "Collecting domain predicates" crlf)
    (bind ?pred-names (create$))
    (bind ?param-counts (create$))
    (bind ?param-names (create$))
    (bind ?param-types (create$))

    (do-for-all-facts ((?dp domain-predicate))
            TRUE
        (printout info "Domain predicate " ?dp:name crlf)
        (bind ?pred-names (insert$ ?pred-names (+ (length$ ?pred-names) 1) (str-cat ?dp:name)))
        (bind ?param-counts (insert$ ?param-counts (+ (length$ ?param-counts) 1) (length$ ?dp:param-names)))
        (loop-for-count (?i 1 (length$ ?dp:param-names))
            (bind ?param-names (insert$ ?param-names (+ (length$ ?param-names) 1) (str-cat (nth$ ?i ?dp:param-names))))
            (bind ?param-types (insert$ ?param-types (+ (length$ ?param-types) 1) (str-cat (nth$ ?i ?dp:param-types))))
        )
    )
 
    (cx-rl-interfaces-get-domain-predicates-response-set-field ?response "predicatenames" ?pred-names)
    (cx-rl-interfaces-get-domain-predicates-response-set-field ?response "paramcounts" ?param-counts)
    (cx-rl-interfaces-get-domain-predicates-response-set-field ?response "paramnames" ?param-names)
    (cx-rl-interfaces-get-domain-predicates-response-set-field ?response "paramtypes" ?param-types)
)