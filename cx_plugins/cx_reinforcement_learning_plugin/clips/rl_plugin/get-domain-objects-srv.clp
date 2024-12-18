(defrule get-domain-objects-service-init
" Create a service returning a list of all domain objects for a certain type in the clips environment."
    (not (cx-rl-interfaces-get-domain-objects-service (name "get_domain_objects")))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-get-domain-objects-create-service "get_domain_objects")
    (printout info "Created service for /get_domain_objects" crlf)
)

(deffunction cx-rl-interfaces-get-domain-objects-service-callback (?service-name ?request ?response)
    (bind ?t (cx-rl-interfaces-get-domain-objects-request-get-field ?request "type"))
    (printout info "Collecting domain objects of type " ?t crlf)
    (bind ?obj-list (create$))
    (do-for-all-facts ((?do domain-object))
            (eq ?do:type (sym-cat ?t))
        (bind ?obj-list (insert$ ?obj-list 1 (str-cat ?do:name)))
    )
    (if (eq ?obj-list (create$)) then
        (bind ?obj-list (insert$ ?obj-list 1 "Not found"))
    )

    (cx-rl-interfaces-get-domain-objects-response-set-field ?response "objects" ?obj-list)
)