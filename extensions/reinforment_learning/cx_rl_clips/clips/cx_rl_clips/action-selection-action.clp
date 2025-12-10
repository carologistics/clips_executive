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

(defrule cx-rl-action-selection-action-server-init
  "Create an action server handling the selection of a given action and collection of its reward after it has finished"
  (cx-rl-node (name ?node-name))
  (not (cx-rl-interfaces-action-selection-server (name ?name&:(eq ?name (str-cat ?node-name "/action_selection")))))
  (not (executive-finalize))
=>
  (bind ?name (str-cat ?node-name "/action_selection"))
  (cx-rl-interfaces-action-selection-create-server ?name)
  (printout ?*CX-RL-LOG-LEVEL* "Created server for " ?name crlf)
)

(deffunction cx-rl-interfaces-action-selection-handle-goal-callback (?server ?goal ?uuid)
  (printout ?*CX-RL-LOG-LEVEL* ?server " callback (goal " ?goal " ; id " ?uuid " )" crlf)
  (return 2)
)

(deffunction cx-rl-interfaces-action-selection-cancel-goal-callback (?server ?goal ?goal-handle)
  (return 1)
)

(defrule cx-rl-action-selection-goal-accepted-start
  (cx-rl-node (name ?node-name))
  (cx-rl-interfaces-action-selection-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  (test (str-index ?node-name ?server))
  (not (rl-ros-action-meta-action-selection (node ?node-name) (uuid ?uuid&:(eq ?uuid (cx-rl-interfaces-action-selection-server-goal-handle-get-goal-id ?ptr)))))
=>
  (if (not (cx-rl-interfaces-action-selection-server-goal-handle-is-canceling ?ptr)) then
      (bind ?goal (cx-rl-interfaces-action-selection-server-goal-handle-get-goal ?ptr))
      (bind ?actionid (cx-rl-interfaces-action-selection-goal-get-field ?goal "actionid"))
      (bind ?uuid (cx-rl-interfaces-action-selection-server-goal-handle-get-goal-id ?ptr))
      (assert (rl-ros-action-meta-action-selection (node ?node-name) (uuid ?uuid) (action-id (sym-cat ?actionid))))
      (bind ?feedback (cx-rl-interfaces-action-selection-feedback-create))
      (cx-rl-interfaces-action-selection-feedback-set-field ?feedback "feedback" "Action selection fact asserted")
      (cx-rl-interfaces-action-selection-server-goal-handle-publish-feedback ?ptr ?feedback)
      (cx-rl-interfaces-action-selection-feedback-destroy ?feedback)
  else
      (printout ?*CX-RL-LOG-LEVEL* "Goal immediately canceled" crlf)
  )
)

(defrule cx-rl-action-selection-abort
  (cx-rl-node (name ?node-name))
  ?ag <- (cx-rl-interfaces-action-selection-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  (test (str-index ?node-name ?server))
  ?as <- (rl-ros-action-meta-action-selection (node ?node-name) (abort-action TRUE) (action-id ?actionid) (uuid ?uuid&:(eq ?uuid (cx-rl-interfaces-action-selection-server-goal-handle-get-goal-id ?ptr))))
=>
  (printout ?*CX-RL-LOG-LEVEL* "ResetCX: Aborting action " ?actionid crlf)
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

(defrule cx-rl-action-selection-finished
  (cx-rl-node (name ?node-name))
  ?ag <- (cx-rl-interfaces-action-selection-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  (test (str-index ?node-name ?server))
  (rl-action (node ?node-name) (id ?action-id) (is-finished TRUE) (reward ?reward))
  ?as <- (rl-ros-action-meta-action-selection (node ?node-name) (action-id ?action-id)
    (uuid ?uuid&:(eq ?uuid (cx-rl-interfaces-action-selection-server-goal-handle-get-goal-id ?ptr)))
    (abort-action FALSE)
  )
=>
  (bind ?done FALSE)
  (do-for-fact ((?end rl-episode-end)) TRUE
    (bind ?done TRUE)
  )
  (printout ?*CX-RL-LOG-LEVEL* "rl-action finished for action " ?action-id crlf)
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

(defrule cx-rl-action-selection-server-cleanup
  (executive-finalize)
  (cx-rl-interfaces-action-selection-server (name ?server))
=>
  (cx-rl-interfaces-action-selection-destroy-server ?server)
)

(defrule cx-rl-action-selection-accepted-goal-cleanup
  (executive-finalize)
  ?ag <- (cx-rl-interfaces-action-selection-accepted-goal (server-goal-handle-ptr ?ptr))
=>
  (cx-rl-interfaces-action-selection-server-goal-handle-destroy ?ptr)
  (retract ?ag)
)
