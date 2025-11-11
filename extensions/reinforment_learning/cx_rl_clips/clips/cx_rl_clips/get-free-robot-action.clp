; Copyright (c) 2025 Carologistics
; SPDX-License-Identifier: Apache-2.0
;
; Licensed under the Apache License, Version 2.0 (the "License");
; you may not use this file except in compliance with the License.
; You may obtain a copy of the License at
;
;     http://www.apache.org/licenses/LICENSE-2.0
;
; Unless required by applicable law or agreed to in writing, software
; distributed under the License is distributed on an "AS IS" BASIS,
; WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; See the License for the specific language governing permissions and
; limitations under the License.

(defrule get-free-robot-action-server-init
    "Create an action server handling requests to determine the robot waiting for a goal selection"
	(cx-rl-node (name ?name))
    (not (cx-rl-interfaces-get-free-robot-server (name ?n&:(eq ?n (str-cat ?name "/get_free_robot")))))
    (not (executive-finalize))
=>
    (cx-rl-interfaces-get-free-robot-create-server (str-cat ?name "/get_free_robot"))
    (printout info "Created server for " ?name "/get_free_robot" crlf)
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
    (do-for-all-facts ((?rw rl-robot))
            (eq ?rw:waiting TRUE)
        (bind ?r ?rw:name)
        (do-for-all-facts ((?a rl-action))
                (and    (eq ?a:is-selected FALSE)
                        (eq ?a:assigned-to ?r))
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
    (declare (salience ?*SALIENCE-RL-FIRST*))
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
