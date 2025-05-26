(defrule get-observable-objects-service-init
" Create a service returning a list of all observable objects for a certain type in the clips environment."
    (not (cx-rl-interfaces-get-observable-objects-service (name "get_observable_objects")))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-get-observable-objects-create-service "get_observable_objects")
    (printout info "Created service for /get_observable_objects" crlf)
)

(deffunction cx-rl-interfaces-get-observable-objects-service-callback (?service-name ?request ?response)
    (bind ?t (cx-rl-interfaces-get-observable-objects-request-get-field ?request "type"))
    (printout info "Collecting cxrl observable objects of type " ?t crlf)
    (bind ?obj-list (create$))
    (do-for-all-facts ((?do rl-observable-object))
            (eq ?do:type (sym-cat ?t))
        (bind ?obj-list (insert$ ?obj-list 1 (str-cat ?do:name)))
    )
    (if (eq ?obj-list (create$)) then
        (bind ?obj-list (insert$ ?obj-list 1 "Not found"))
    )

    (cx-rl-interfaces-get-observable-objects-response-set-field ?response "objects" ?obj-list)
)