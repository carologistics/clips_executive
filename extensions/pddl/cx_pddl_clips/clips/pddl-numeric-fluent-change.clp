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

(deftemplate pddl-numeric-fluent-change
" Interface for numeric-fluents.clp
  Assert a fact of this type in order to indicate that a numeric fluent needs
  to be added to/removed from a pddl instance.
  Acts as a transient layer to pddl-numeric-fluent facts to make sure the CLIPS
  representation stays consistant with the externally managed pddl instance.
  @slot instance: pddl instance to add the fluent to.
  @slot name: name of the fluent.
  @slot params: parameters of the fluent.
  @slot value: value of the fluent.
  Slots set automatically:
  @slot request-id: id of the associated ros service request
  @slot state:
   - PENDING: The fluent was not added yet.
   - WAITING: The fluent is about to be added and is waiting for confirmation.
   - ERROR: The fluent might not have been fetched due to an error.
   - ON-HOLD: Unused state that can be set in order to defer the fluent update
     to a later time (by switching it manually to PENDING).
"
  (slot instance (type SYMBOL))
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL) (default (create$)))
  (slot value (type FLOAT))
  (slot request-id (type INTEGER))
  (slot state (type SYMBOL) (allowed-values PENDING WAITING ERROR ON-HOLD) (default PENDING))
)

(defrule pddl-numeric-fluent-change-request
  (declare (salience ?*PRIORITY-PDDL-FLUENTS*))
  (pddl-numeric-fluent-change (instance ?instance) (state PENDING))
  ?pi-f <- (pddl-instance (name ?instance) (state LOADED) (busy-with FALSE))
  (pddl-manager (node ?node))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/set_functions"))) (type ?type))
  (not (pddl-service-request-meta (service ?s) (meta ?instance)))
  =>
  (bind ?function-msgs (create$))
  (do-for-all-facts ((?ppf pddl-numeric-fluent-change)) (and (eq ?ppf:state PENDING) (eq ?ppf:instance ?instance))
    (bind ?function-msg (ros-msgs-create-message "cx_pddl_msgs/msg/Function"))
    (ros-msgs-set-field ?function-msg "pddl_instance" ?ppf:instance)
    (ros-msgs-set-field ?function-msg "name" ?ppf:name)
    (ros-msgs-set-field ?function-msg "args" ?ppf:params)
    (ros-msgs-set-field ?function-msg "value" ?ppf:value)
    (bind ?function-msgs (create$ ?function-msgs ?function-msg))
    (modify ?ppf (state WAITING))
  )
  (if (> (length$ ?function-msgs) 0) then
    (bind ?new-req (ros-msgs-create-request ?type))
    (ros-msgs-set-field ?new-req "functions" ?function-msgs)
    (bind ?id (ros-msgs-async-send-request ?new-req ?s))
    (if ?id then
      (modify ?pi-f (busy-with NUMERIC-FLUENTS))
      (assert (pddl-service-request-meta (service ?s) (request-id ?id) (meta (sym-cat ?instance))))
     else
      (printout error "Sending of request failed, is the service " ?s " running?" crlf)
    )
    (ros-msgs-destroy-message ?new-req)
    (foreach ?msg ?function-msgs
      (ros-msgs-destroy-message ?msg)
    )
  )
)

(defrule pddl-numeric-fluent-change-process-response
" Process a response to the /set_functions service by removing the respective pddl-numeric-fluent facts and clean up the associated pending facts afterwards.
"
  (pddl-manager (node ?node))
  ?pi-f <- (pddl-instance (name ?instance) (busy-with NUMERIC-FLUENTS))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/set_functions"))))
  ?req-f <- (pddl-service-request-meta (service ?s) (meta ?instance) (request-id ?id))
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  =>
  (modify ?pi-f (busy-with FALSE))
  (bind ?success (ros-msgs-get-field ?ptr "success"))
  (bind ?error (ros-msgs-get-field ?ptr "error"))
  (if ?success then
    (printout debug "Successfully set functions" crlf)
    (delayed-do-for-all-facts ((?ppf pddl-numeric-fluent-change)) (and (eq ?ppf:state WAITING) (eq ?ppf:instance ?instance))
      (if (not (do-for-fact ((?fluent pddl-numeric-fluent)) (and (eq ?fluent:name ?ppf:name) (eq ?fluent:params ?ppf:params))
        (modify ?fluent (value ?ppf:value)))) then
        (assert (pddl-numeric-fluent (name ?ppf:name) (instance ?instance) (params ?ppf:params) (value ?ppf:value)))
      )
      (retract ?ppf)
    )
   else
    (printout error "Failed to set numeric fluents \"" ?instance "\":" ?error crlf)
    ; TODO: how to deal with failed removing of fluents
    (delayed-do-for-all-facts ((?ppf pddl-numeric-fluent-change)) (and ?ppf:request-sent (eq ?ppf:instance ?instance) ?ppf:delete)
      (modify ?ppf (error ?error) (state ERROR))
    )
  )
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
  (retract ?req-f)
)
