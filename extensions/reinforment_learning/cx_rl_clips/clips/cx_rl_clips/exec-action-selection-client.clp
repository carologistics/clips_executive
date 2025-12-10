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


(defrule cx-rl-exec-action-selection-request
  (cx-rl-node (name ?node) (mode EXECUTION))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/exec_action_selection"))) (type ?type))
  (not (rl-action-request-meta (service ?s)))
  (rl-current-action-space (state DONE))
=>
  (printout ?*CX-RL-LOG-LEVEL* "Action selection demand found" crlf)
  (bind ?state-string (cx-rl-create-observation-string ?node))

  (bind ?action-list (create$))
  (do-for-all-facts ((?action rl-action))
          (eq ?action:is-selected FALSE)
      (bind ?action-string (str-cat ?action:id "|" ?action:name))
      (printout ?*CX-RL-LOG-LEVEL* "Executable action: " ?action-string crlf)
      (bind ?action-list (insert$ ?action-list 1 ?action-string))
  )

  (printout ?*CX-RL-LOG-LEVEL* "Requesting action selection" crlf)
  (bind ?new-req (ros-msgs-create-request ?type))
  (ros-msgs-set-field ?new-req "state" ?state-string)
  (ros-msgs-set-field ?new-req "actions" ?action-list)
  (bind ?id (ros-msgs-async-send-request ?new-req ?s))
  (if ?id then
    (assert (rl-action-request-meta (node ?node) (service ?s) (request-id ?id)))
  )
  (ros-msgs-destroy-message ?new-req)
)

(defrule cx-rl-exec-action-selection-response-received
  (cx-rl-node (name ?node) (mode EXECUTION))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/exec_action_selection"))) (type ?type))
  ?msg-fact <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  ?req-meta <- (rl-action-request-meta (node ?node) (service ?s) (request-id ?id))
=>
  (bind ?action-id (ros-msgs-get-field ?ptr "actionid"))
  (printout ?*CX-RL-LOG-LEVEL* "Received actionid from " ?s ": " ?action-id crlf)
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-fact)
  (modify ?req-meta (action-id ?action-id))
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
