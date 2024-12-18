(defrule get-free-robot-action-server-init
    "Create an action server handling requests to determine the robot waiting for a goal selection"
    (not (cx-rl-interfaces-get-free-robot-server (name "get_free_robot")))
    (not (executive-finalize))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-get-free-robot-create-server "get_free_robot")
    (printout info "Created server for /get_free_robot" crlf)
)

(deffunction cx-rl-interfaces-get-free-robot-handle-goal-callback (?server ?goal ?uuid)
    (printout blue ?server " callback (goal " ?goal " ; id " ?uuid " )" crlf)
    (return 2)
)

(deffunction cx-rl-interfaces-get-free-robot-cancel-goal-callback (?server ?goal ?goal-handle)
    (return 1)
)

(deftemplate get-free-robot
    (slot uuid (type STRING))
    (slot robot (type STRING))
    (slot last-search (type FLOAT))
    (slot found (type SYMBOL)
                (allowed-values TRUE FALSE))
)

(defrule get-free-robot-goal-accepted-start-search
    (cx-rl-interfaces-get-free-robot-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
    (not (get-free-robot (uuid ?uuid&:(eq ?uuid (cx-rl-interfaces-get-free-robot-server-goal-handle-get-goal-id ?ptr)))))
    (time ?now)
=>
    (if (not (cx-rl-interfaces-get-free-robot-server-goal-handle-is-canceling ?ptr)) then
        (bind ?uuid (cx-rl-interfaces-get-free-robot-server-goal-handle-get-goal-id ?ptr))
        (assert (get-free-robot (uuid ?uuid) (robot "") (last-search ?now) (found FALSE)))
    else
        (printout error "Goal immediately canceled" crlf)
    )
)

(defrule get-free-robot-search
    (cx-rl-interfaces-get-free-robot-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
    ?gfr <- (get-free-robot (uuid ?uuid) (robot "") (last-search ?last) (found FALSE))
    (time ?now&:(> (- ?now ?last) 1))
    (test (eq ?uuid (cx-rl-interfaces-get-free-robot-server-goal-handle-get-goal-id ?ptr)))
=>  
    (bind ?robot NONE)
    (do-for-all-facts ((?df domain-fact))
            (eq ?df:name robot-waiting)
        (bind ?r (nth$ 1 ?df:param-values))
        (do-for-all-facts ((?g goal))
                (and    (eq ?g:mode FORMULATED)
                        (eq ?g:is-executable TRUE)
                        (eq ?g:assigned-to ?r))
            (bind ?robot ?r)
            (break)
        )
    )
    (if (eq ?robot NONE) then
        (bind ?feedback (cx-rl-interfaces-get-free-robot-feedback-create))
        (cx-rl-interfaces-get-free-robot-feedback-set-field ?feedback "feedback" "No free robot found, retrying...")
        (cx-rl-interfaces-get-free-robot-server-goal-handle-publish-feedback ?ptr ?feedback)
        (cx-rl-interfaces-get-free-robot-feedback-destroy ?feedback)
        (modify ?gfr (last-search ?now))
    else
        (modify ?gfr (robot (str-cat ?robot)) (last-search ?now) (found TRUE))
    )
)

(defrule get-free-robot-abort
    (declare (salience ?*SALIENCE-FIRST*))
    (abort-get-free-robot)
    ?ag <- (cx-rl-interfaces-get-free-robot-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
    ?gfr <- (get-free-robot (robot ?robot) (last-search ?last) (found ?found) (uuid ?uuid))
    (test (eq ?uuid (cx-rl-interfaces-get-free-robot-server-goal-handle-get-goal-id ?ptr)))
=>
    (printout info "ResetCX: Aborting robot search" crlf)
    (bind ?result (cx-rl-interfaces-get-free-robot-result-create))
    (cx-rl-interfaces-get-free-robot-result-set-field ?result "robot" "Aborted")
    (cx-rl-interfaces-get-free-robot-server-goal-handle-abort ?ptr ?result)
    (cx-rl-interfaces-get-free-robot-result-destroy ?result)
    (cx-rl-interfaces-get-free-robot-server-goal-handle-destroy ?ptr)
    (retract ?gfr)
    (retract ?ag)
)

(defrule get-free-robot-search-done
    ?ag <- (cx-rl-interfaces-get-free-robot-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
    ?gfr <- (get-free-robot (robot ?robot) (last-search ?last) (found TRUE) (uuid ?uuid))
    (test (eq ?uuid (cx-rl-interfaces-get-free-robot-server-goal-handle-get-goal-id ?ptr)))
=>
    (printout green "Free robot found: " ?robot crlf)
    (bind ?result (cx-rl-interfaces-get-free-robot-result-create))
    (cx-rl-interfaces-get-free-robot-result-set-field ?result "robot" ?robot)
    (cx-rl-interfaces-get-free-robot-server-goal-handle-succeed ?ptr ?result)
    (cx-rl-interfaces-get-free-robot-result-destroy ?result)
    (cx-rl-interfaces-get-free-robot-server-goal-handle-destroy ?ptr)
    (retract ?gfr)
    (retract ?ag)
)

(defrule get-free-robot-server-cleanup
    (executive-finalize)
    (cx-rl-interfaces-get-free-robot-server (name ?server))
=>
    (cx-rl-interfaces-get-free-robot-destroy-server ?server)
)

(defrule get-free-robot-accepted-goal-cleanup
    (executive-finalize)
    ?ag <- (cx-rl-interfaces-get-free-robot-accepted-goal (server-goal-handle-ptr ?ptr))
=>
    (cx-rl-interfaces-get-free-robot-server-goal-handle-destroy ?ptr)
    (retract ?ag)
)