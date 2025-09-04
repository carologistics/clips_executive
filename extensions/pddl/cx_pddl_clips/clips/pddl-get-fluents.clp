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

(deftemplate pddl-get-fluents
" Interface for get-fluents.clp
  Assert a fact of this type in order to fetch all positive boolean fluents
  of a given pddl instance with the external pddl manager.
  This results in the automatic assertion of all positive boolean fluents
  (pddl-fluent template facts) currently present in the given pddl instance.
  @slot instance: pddl instance from which the fluents are fetched.
  Slots set automatically:
  @slot state:
   - PENDING: The fluents were not fetched yet.
   - DONE: The fluents were successfully retrieved
   - ERROR: The fluents were not fetched due to an error.
  @slot error: provide information on encountered errors.
"
  (slot instance (type SYMBOL))
  (slot state (type SYMBOL) (allowed-values PENDING DONE ERROR) (default PENDING))
  (slot error (type STRING))
)

(defrule pddl-get-fluents-request
  (pddl-get-fluents (instance ?instance) (state PENDING))
  (pddl-manager (node ?node))
  ?pi-f <- (pddl-instance (name ?instance) (state LOADED) (busy-with FALSE))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/get_fluents"))) (type ?type))
  (not (pddl-service-request-meta (service ?s)))
  (time ?any-time) ; used to continuously attempt to request the service until success
  =>
  (bind ?new-req (ros-msgs-create-request ?type))
  (ros-msgs-set-field ?new-req "pddl_instance" ?instance)
  (bind ?id (ros-msgs-async-send-request ?new-req ?s))
  (if ?id then
    (modify ?pi-f (busy-with GET-FLUENTS))
    (assert (pddl-service-request-meta (service ?s) (request-id ?id) (meta ?instance)))
   else
    (printout error "Sending of request failed, is the service " ?s " running?" crlf)
  )
  (ros-msgs-destroy-message ?new-req)
)

(defrule pddl-get-fluents-response-received
" Get response, read it and delete."
  ?get-facts-f <- (pddl-get-fluents (instance ?instance) (state PENDING))
  (pddl-manager (node ?node))
  ?pi-f <- (pddl-instance (name ?instance) (busy-with GET-FLUENTS))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/get_fluents"))) (type ?type))
  ?msg-f <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  ?req-meta <- (pddl-service-request-meta (service ?s) (request-id ?id) (meta ?instance))
=>
  (modify ?pi-f (busy-with FALSE))
  (bind ?success (ros-msgs-get-field ?ptr "success"))
  (bind ?error (ros-msgs-get-field ?ptr "error"))
  (if ?success then
    (bind ?fluents (ros-msgs-get-field ?ptr "fluents"))
    (foreach ?fluent ?fluents
      (bind ?instance (sym-cat (ros-msgs-get-field ?fluent "pddl_instance")))
      (bind ?name (sym-cat (ros-msgs-get-field ?fluent "name")))
      (bind ?args (ros-msgs-get-field ?fluent "args"))
      (bind ?arg-syms (create$))
      (foreach ?arg ?args
        (bind ?arg-syms (create$ ?arg-syms (sym-cat ?arg)))
      )
      (assert (pddl-fluent (name ?name) (params ?arg-syms) (instance ?instance)))
    )
    (modify ?get-facts-f (state DONE))
   else
    (modify ?get-facts-f (state ERROR) (error ?error))
    (printout error "Failed to get fluents (" ?instance "):" ?error crlf)
  )
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-f)
  (retract ?req-meta)
)
