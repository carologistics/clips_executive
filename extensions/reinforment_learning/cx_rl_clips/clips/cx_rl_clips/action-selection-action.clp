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

(defrule action-selection-action-server-init
    "Create an action server handling the selection of a given action and collection of its reward after it has finished"
    (cx-rl-node (name ?name))
    (not (cx-rl-interfaces-action-selection-server (name ?name&:(eq ?name (str-cat ?name "/action_selection")))))
    (not (executive-finalize))
=>
    (cx-rl-interfaces-action-selection-create-server (str-cat ?*CX-RL-NODE-NAME* "action_selection"))
    (printout info "Created server for /action_selection" crlf)
)

(deffunction cx-rl-interfaces-action-selection-handle-goal-callback (?server ?goal ?uuid)
    (printout blue ?server " callback (goal " ?goal " ; id " ?uuid " )" crlf)
    (return 2)
)

(deffunction cx-rl-interfaces-action-selection-cancel-goal-callback (?server ?goal ?goal-handle)
    (return 1)
)



(defrule action-selection-goal-accepted-start
    (cx-rl-interfaces-action-selection-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
    (not (rl-ros-action-meta (uuid ?uuid&:(eq ?uuid (cx-rl-interfaces-action-selection-server-goal-handle-get-goal-id ?ptr)))))
=>
    (if (not (cx-rl-interfaces-action-selection-server-goal-handle-is-canceling ?ptr)) then
        (bind ?goal (cx-rl-interfaces-action-selection-server-goal-handle-get-goal ?ptr))
        (bind ?actionid (cx-rl-interfaces-action-selection-goal-get-field ?goal "actionid"))
        (bind ?uuid (cx-rl-interfaces-action-selection-server-goal-handle-get-goal-id ?ptr))
        (assert (rl-ros-action-meta (uuid ?uuid) (action-id (sym-cat ?actionid))))
        (bind ?feedback (cx-rl-interfaces-action-selection-feedback-create))
        (cx-rl-interfaces-action-selection-feedback-set-field ?feedback "feedback" "Action selection fact asserted")
        (cx-rl-interfaces-action-selection-server-goal-handle-publish-feedback ?ptr ?feedback)
        (cx-rl-interfaces-action-selection-feedback-destroy ?feedback)
    else
        (printout error "Goal immediately canceled" crlf)
    )
)

(defrule action-selection-abort
    (declare (salience ?*SALIENCE-RL-FIRST*))
    (abort-all-action-selections)
    ?ag <- (cx-rl-interfaces-action-selection-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
    ?as <- (rl-ros-action-meta (action-id ?actionid) (uuid ?uuid&:(eq ?uuid (cx-rl-interfaces-action-selection-server-goal-handle-get-goal-id ?ptr))))
=>
    (printout info "ResetCX: Aborting action " ?actionid crlf)
    (bind ?result (cx-rl-interfaces-action-selection-result-create))
    (cx-rl-interfaces-action-selection-result-set-field ?result "actionid" (str-cat ?actionid))
    (cx-rl-interfaces-action-selection-result-set-field ?result "reward" 0)
    (cx-rl-interfaces-action-selection-result-set-field ?result "info" "Aborted")
    (cx-rl-interfaces-action-selection-server-goal-handle-abort ?ptr ?result)
    (cx-rl-interfaces-action-selection-result-destroy ?result)
    (cx-rl-interfaces-action-selection-server-goal-handle-destroy ?ptr)
    (retract ?as)
    (retract ?ag)
)

(defrule action-selection-finished
  ?ag <- (cx-rl-interfaces-action-selection-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  (rl-action (id ?action-id) (is-finished TRUE) (reward ?reward))
  ?as <- (rl-ros-action-meta (action-id ?action-id) (uuid ?uuid&:(eq ?uuid (cx-rl-interfaces-action-selection-server-goal-handle-get-goal-id ?ptr))))
=>
  (bind ?done FALSE)
  (do-for-fact ((?end rl-episode-end)) TRUE
    (bind ?done TRUE)
  )
  (printout green "rl-action finished for action " ?action-id crlf)
  (if (eq ?done TRUE) then (bind ?info "Done") else (bind ?info ""))
  (bind ?result (cx-rl-interfaces-action-selection-result-create))
  (cx-rl-interfaces-action-selection-result-set-field ?result "actionid" (str-cat ?action-id))
  (cx-rl-interfaces-action-selection-result-set-field ?result "reward" ?reward)
  (cx-rl-interfaces-action-selection-result-set-field ?result "info" ?info)
  (cx-rl-interfaces-action-selection-server-goal-handle-succeed ?ptr ?result)
  (cx-rl-interfaces-action-selection-result-destroy ?result)
  (cx-rl-interfaces-action-selection-server-goal-handle-destroy ?ptr)
  (retract ?as)
  (retract ?ag)
)

(defrule action-selection-server-cleanup
    (executive-finalize)
    (cx-rl-interfaces-action-selection-server (name ?server))
=>
    (cx-rl-interfaces-action-selection-destroy-server ?server)
)

(defrule action-selection-accepted-goal-cleanup
    (executive-finalize)
    ?ag <- (cx-rl-interfaces-action-selection-accepted-goal (server-goal-handle-ptr ?ptr))
=>
    (cx-rl-interfaces-action-selection-server-goal-handle-destroy ?ptr)
    (retract ?ag)
)
