(defrule pddl-init
    (not (pddl-services-loaded))
    =>
    ; create clints for all services
    (bind ?services (create$
        add_fluents AddFluents
        add_pddl_instance AddPddlInstance
        get_fluents GetFluents
        set_goals SetGoals
    ))
    (bind ?index 1)
    (bind ?length (length$ ?services))
    (while (< ?index ?length)
        (bind ?service-name (nth$ ?index ?services))
        (bind ?service-type (nth$ (+ ?index 1) ?services))
        (ros-msgs-create-client
            (str-cat "/pddl_manager" "/" ?service-name)
            (str-cat "pddl_msgs/srv/" ?service-type)
        )
        (bind ?index (+ ?index 2))
    )
    (assert (pddl-services-loaded))
)

(defrule pddl-init-plan-client
    (not (pddl-planning-client-created))
    =>
    (pddl-msgs-plan-temporal-create-client (str-cat "/pddl_manager" "/temp_plan"))
    (assert (pddl-planning-client-created))
)

; ---------------- SETUP INSTANCE ------------------

(defrule pddl-add-instance
	(ros-msgs-client (service ?service&:(eq ?service (str-cat "/pddl_manager" "/add_pddl_instance"))) (type ?type))
	(not (pddl-loaded))
    (pddl-services-loaded)
	=>
	(bind ?new-req (ros-msgs-create-request ?type))
	(ros-msgs-set-field ?new-req "name" "test") ;instance of name test
	(bind ?share-dir (ament-index-get-package-share-directory "pddl_agent_example"))
	(ros-msgs-set-field ?new-req "directory" (str-cat ?share-dir "/pddl"))
	(ros-msgs-set-field ?new-req "domain_file" "domain.pddl")
	(ros-msgs-set-field ?new-req "problem_file" "problem.pddl")
	(bind ?id (ros-msgs-async-send-request ?new-req ?service))
	(assert (pddl-loaded))
	(ros-msgs-destroy-message ?new-req)
)

(defrule pddl-add-instance-result
	(ros-msgs-client (service ?s&:(eq ?s (str-cat "/pddl_manager" "/add_pddl_instance"))) (type ?type))
	?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
	=>
    (bind ?success (ros-msgs-get-field ?ptr "success"))
    (bind ?error (ros-msgs-get-field ?ptr "error"))
    (if ?success then
        (printout t "PDDL instance added" crlf)
    else
        (printout error "Failed to set problem instance" ?error crlf)
    )
    (ros-msgs-destroy-message ?ptr)
    (retract ?msg-f)
)

; ---------------- GET CURRENT FLUENTS ------------------

(defrule pddl-get-fluents
    (ros-msgs-client (service ?s&:(eq ?s (str-cat "/pddl_manager" "/get_fluents"))) (type ?type))
    (pddl-loaded)
    (not (pddl-fluents-requested))
    =>
    (bind ?new-req (ros-msgs-create-request ?type))
    (ros-msgs-set-field ?new-req "pddl_instance" "test")
    (bind ?id (ros-msgs-async-send-request ?new-req ?s))
    (if ?id then
        (printout t "Requested Fluents" crlf)
        (assert (pddl-fluents-requested))
    else
        (printout error "Sending of request failed, is the service " ?s " running?" crlf)
    )
    (ros-msgs-destroy-message ?new-req)
)

(defrule pddl-get-fluents-result
" Get response, read it and delete."
    (ros-msgs-client (service ?s&:(eq ?s (str-cat "/pddl_manager" "/get_fluents"))) (type ?type))
    ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
    (pddl-fluents-requested)
    =>
    (bind ?success (ros-msgs-get-field ?ptr "success"))
    (bind ?error (ros-msgs-get-field ?ptr "error"))
    (if ?success then
        (printout t "Got fluents from current instance" crlf)
        (bind ?fluents (ros-msgs-get-field ?ptr "fluents"))
        (foreach ?fluent ?fluents
        (bind ?instance (sym-cat (ros-msgs-get-field ?fluent "pddl_instance")))
        (bind ?name (sym-cat (ros-msgs-get-field ?fluent "name")))
        (bind ?args (ros-msgs-get-field ?fluent "args"))
        (printout t ?name ?args crlf)
        )
    else
        (printout error "Failed to get fluents (" "test" "):" ?error crlf)
    )
    (ros-msgs-destroy-message ?ptr)
    )


; ---------------- SET GOAL ------------------

(defrule pddl-set-goal
    (pddl-fluents-requested)
    (not (pddl-goals-set))
    (ros-msgs-client (service ?s&:(eq ?s (str-cat "/pddl_manager" "/set_goals"))) (type ?type))
    =>
    (bind ?new-req (ros-msgs-create-request ?type))
    (bind ?fluent-goal-msgs (create$))

    (bind ?fluent-msg (ros-msgs-create-message "pddl_msgs/msg/Fluent"))
    (ros-msgs-set-field ?fluent-msg "pddl_instance" "test")
    (ros-msgs-set-field ?fluent-msg "name" "on")
    (ros-msgs-set-field ?fluent-msg "args" (create$ "a" "b"))
    (bind ?fluent-goal-msgs (create$ ?fluent-goal-msgs ?fluent-msg))
    (bind ?fluent-msg (ros-msgs-create-message "pddl_msgs/msg/Fluent"))
    (ros-msgs-set-field ?fluent-msg "pddl_instance" "test")
    (ros-msgs-set-field ?fluent-msg "name" "on")
    (ros-msgs-set-field ?fluent-msg "args" (create$ "b" "c"))
    (bind ?fluent-goal-msgs (create$ ?fluent-goal-msgs ?fluent-msg))

    (ros-msgs-set-field ?new-req "fluents" ?fluent-goal-msgs)
    (bind ?id (ros-msgs-async-send-request ?new-req ?s))
    (if ?id then
        (printout t "Requested to set goals" crlf)
    else
        (printout error "Sending of request failed, is the service " ?s " running?" crlf)
    )
    (foreach ?msg ?fluent-goal-msgs
        (ros-msgs-destroy-message ?msg)
    )
    (ros-msgs-destroy-message ?new-req)
    (assert (pddl-goals-set))
)

(defrule pddl-set-goal-result
" Get response, read it and delete."
    (ros-msgs-client (service ?s&:(eq ?s (str-cat "/pddl_manager" "/set_goals"))) (type ?type))
    ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
    (pddl-goals-set)
    =>
    (bind ?success (ros-msgs-get-field ?ptr "success"))
    (bind ?error (ros-msgs-get-field ?ptr "error"))
    (if ?success then
        (printout t "Goals set successfully" crlf)
    else
        (printout error "Failed to set goals (" "test" "):" ?error crlf)
    )
    (ros-msgs-destroy-message ?ptr)
    (retract ?msg-f)
)
; ---------------- PLAN FOR INSTANCE ------------------



(defrule pddl-plan
    (pddl-msgs-plan-temporal-client (server ?server&:(eq ?server "/pddl_manager/temp_plan")))
    (not (planned))
    =>
    (printout green "Start planning" crlf)
    (bind ?goal (pddl-msgs-plan-temporal-goal-create))
    (pddl-msgs-plan-temporal-goal-set-field ?goal "pddl_instance" "test")
    (pddl-msgs-plan-temporal-goal-set-field ?goal "goal_instance" "base")
    (pddl-msgs-plan-temporal-send-goal ?goal ?server)
    (assert (planned))
)

(defrule pddl-plan-result
    ?wr-f <- (pddl-msgs-plan-temporal-wrapped-result (server "/pddl_manager/temp_plan") (code SUCCEEDED) (result-ptr ?res-ptr))
    =>
    (bind ?plan-found (pddl-msgs-plan-temporal-result-get-field ?res-ptr "success"))
    (printout green "planning done" crlf)
    (if ?plan-found then
        (bind ?plan (pddl-msgs-plan-temporal-result-get-field ?res-ptr "actions"))
        (foreach ?action ?plan
        (bind ?name (sym-cat (pddl-msgs-timed-plan-action-get-field ?action "name")))
        (bind ?args (pddl-msgs-timed-plan-action-get-field ?action "args"))
        (bind ?ps-time (pddl-msgs-timed-plan-action-get-field ?action "start_time"))
        (bind ?p-duration (pddl-msgs-timed-plan-action-get-field ?action "duration"))
        (printout t ?ps-time "(" ?p-duration ")   " ?name ?args crlf)
        )
    else
        (printout red "plan not found!" crlf)
    )
    (pddl-msgs-plan-temporal-result-destroy ?res-ptr)
)
