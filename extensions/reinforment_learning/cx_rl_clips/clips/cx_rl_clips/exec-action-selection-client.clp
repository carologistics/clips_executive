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
    (printout info "Action selection demand found" crlf)
    (bind ?state-string (create-observation-string))

    (bind ?action-list (create$))
    (do-for-all-facts ((?action rl-action))
            (eq ?action:is-selected FALSE)
        (bind ?action-string (str-cat ?action:id "|" ?action:name))
        (printout info "Executable action: " ?action-string crlf)
        (bind ?action-list (insert$ ?action-list 1 ?action-string))
    )

    (printout info "Requesting action selection" crlf)
    (bind ?new-req (ros-msgs-create-request ?type))
    (ros-msgs-set-field ?new-req "state" ?state-string)
    (ros-msgs-set-field ?new-req "actions" ?action-list)
    (bind ?id (ros-msgs-async-send-request ?new-req ?s))
    (if ?id then
      (assert (rl-action-request-meta (service ?s) (request-id ?id)))
  )
    (ros-msgs-destroy-message ?new-req)
)

(defrule cx-rl-exec-action-selection-response-received
  (cx-rl-node (name ?node) (mode EXECUTION))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/exec_action_selection"))) (type ?type))
    ?msg-fact <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
   ?req-meta <- (rl-action-request-meta (service ?s) (request-id ?id))
=>
    (bind ?action-id (ros-msgs-get-field ?ptr "actionid"))
    (printout green "Received actionid from " ?s ": " ?action-id crlf)
    (ros-msgs-destroy-message ?ptr)
    (retract ?msg-fact)
	(modify ?req-meta (action-id ?action-id))
)


(defrule rl-action-select-execution
  (declare (salience ?*SALIENCE-RL-SELECTION*))
  (cx-rl-node (mode EXECUTION))
  ?r <- (rl-action-request-meta (action-id ?action-id))
 ?action-space <- (rl-current-action-space (state DONE))
  ?next-action <- (rl-action (id ?action-id) (is-selected FALSE) (assigned-to ?robot))
  ?rw <- (rl-robot (name ?robot) (waiting TRUE))
  =>
  (printout info crlf "CXRL: Selected action " ?action-id  "for robot " ?robot crlf)

  (retract ?r)
  (retract ?action-space)
  (modify ?rw (waiting FALSE))
  (modify ?next-action (is-selected TRUE))
  (delayed-do-for-all-facts ((?a rl-action))
		(eq ?a:is-selected FALSE)
		(retract ?a)
	)
)
