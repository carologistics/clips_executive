; Copyright (c) 2025-2026 Carologistics
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


(defrule cx-rl-exec-action-selection-request
  (not (executive-finalize))
  (cx-rl-node (name ?node) (mode EXECUTION) (model-loaded TRUE))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/exec_action_selection"))) (type ?type))
  (not (rl-action-request-meta (service ?s)))
  (rl-current-action-space (node ?node) (state DONE))
  (time ?now)
=>
  (printout ?*CX-RL-LOG-LEVEL* "Action selection demand found" crlf)

  (bind ?obs-list (create$))
  (do-for-all-facts ((?obs rl-observation)) (eq ?obs:node ?node)
      (bind ?observation-msg (ros-msgs-create-message "cx_rl_interfaces/msg/Observation"))
      (ros-msgs-set-field ?observation-msg "name" ?obs:name)
      (ros-msgs-set-field ?observation-msg "params" ?obs:params)
      (bind ?obs-list (insert$ ?obs-list 1 ?observation-msg))
  )
  (bind ?temp-action-list (create$))
  (bind ?action-list (create$))
  (bind ?action-id-list (create$))
  (do-for-all-facts ((?action rl-action))
          (eq ?action:is-selected FALSE)
      (bind ?action-msg (ros-msgs-create-message "cx_rl_interfaces/msg/Action"))
      (ros-msgs-set-field ?action-msg "name" ?action:name)
      (ros-msgs-set-field ?action-msg "params" ?action:params)
      (bind ?action-list (insert$ ?action-list 1 ?action-msg))
      (bind ?action-id-list (insert$ ?action-id-list 1 ?action:id))
      (bind ?temp-action-list (insert$ ?temp-action-list 1 ?action-msg))
  )

  (printout ?*CX-RL-LOG-LEVEL* "Requesting action selection" crlf)
  (bind ?new-req (ros-msgs-create-request ?type))
  (ros-msgs-set-field ?new-req "observations" ?obs-list)
  (ros-msgs-set-field ?new-req "actions" ?action-list)
  (ros-msgs-set-field ?new-req "action_ids" ?action-id-list)
  (foreach ?action-msg-item ?temp-action-list
    (ros-msgs-destroy-message ?action-msg-item)
  )
  (foreach ?observation-msg-item ?obs-list
    (ros-msgs-destroy-message ?observation-msg-item)
  )
  (bind ?id (ros-msgs-async-send-request ?new-req ?s))
  (if ?id then
    (assert (rl-action-request-meta (node ?node) (service ?s) (request-id ?id)))
   else
    (printout error "Action selection request failed, retrying" crlf)
  )
  (ros-msgs-destroy-message ?new-req)
)

(defrule cx-rl-exec-action-selection-response-received
  (cx-rl-node (name ?node) (mode EXECUTION))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/exec_action_selection"))) (type ?type))
  ?msg-fact <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  ?req-meta <- (rl-action-request-meta (node ?node) (service ?s) (request-id ?id))
=>
  (bind ?action-id (sym-cat (ros-msgs-get-field ?ptr "action_id")))
  (printout ?*CX-RL-LOG-LEVEL* "Received action_id from " ?s ": " ?action-id crlf)
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-fact)
  (modify ?req-meta (action-id ?action-id))
)

(defrule cx-rl-exec-action-selection-action-unknown
  (cx-rl-node (name ?node) (mode EXECUTION))
  ?r <- (rl-action-request-meta (node ?node) (action-id ?action-id&:(neq ?action-id nil)))
  ?action-space <- (rl-current-action-space (node ?node) (state DONE))
  (not (rl-action (node ?node) (id ?action-id)))
  ?rw <- (rl-robot (node ?node) (name ?robot) (waiting TRUE))
  =>
  (printout ?*CX-RL-LOG-LEVEL* crlf "CXRL: Selected action unknown " crlf)
  (retract ?r)
  (retract ?action-space)
  (modify ?rw (waiting FALSE))
  (assert (rl-action (node ?node) (id ?action-id) (assigned-to ?robot) (is-selected TRUE)))
  (delayed-do-for-all-facts ((?a rl-action))
    (and (eq ?a:is-selected FALSE) (eq ?a:node ?node))
    (retract ?a)
  )
)


(defrule rl-action-select-execution
  (cx-rl-node (name ?node) (mode EXECUTION))
  ?r <- (rl-action-request-meta (node ?node) (action-id ?action-id))
  ?action-space <- (rl-current-action-space (node ?node) (state DONE))
  ?next-action <- (rl-action (node ?node) (id ?action-id) (is-selected FALSE) (assigned-to ?robot))
  ?rw <- (rl-robot (node ?node) (name ?robot) (waiting TRUE))
  =>
  (printout ?*CX-RL-LOG-LEVEL* crlf "CXRL: Selected action " ?action-id  "for robot " ?robot crlf)
  (retract ?r)
  (retract ?action-space)
  (modify ?rw (waiting FALSE))
  (modify ?next-action (is-selected TRUE))
  (delayed-do-for-all-facts ((?a rl-action))
    (and (eq ?a:is-selected FALSE) (eq ?a:node ?node))
    (retract ?a)
  )
)

(defrule cx-rl-exec-action-selection-cleanup
  (executive-finalize)
  (cx-rl-node (name ?node))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/exec_action_selection"))) (type ?type))
  =>
  (ros-msgs-destroy-client ?s)
)
