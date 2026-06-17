; Copyright (c) 2026 Carologistics
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

(defrule pddl-plan-start-plan
" Call a planner based on a pddl-plan fact."
  (declare (salience ?*PRIORITY-PDDL-PLAN*))
  (pddl-manager (node ?node))
  (pddl-instance (name ?instance) (state LOADED) (busy-with FALSE))
  (cx-pddl-interfaces-plan-client (server ?server&:(eq ?server (str-cat ?node "/plan"))))
  ?pp <- (pddl-plan (instance ?instance) (goal ?goal) (plan-type ?plan-type) (state PENDING) (output-dir ?dir))
  (not (pddl-plan (state WAITING)))
  =>
  (bind ?plan-kind-int 0)
  (switch ?plan-type
    (case CLASSICAL then (bind ?plan-kind-int 0))
    (case TEMPORAL then (bind ?plan-kind-int 1))
    (case PARTIAL-ORDER then (bind ?plan-kind-int 2))
    (case HIERARCHICAL then (bind ?plan-kind-int 3))
    (case STN then (bind ?plan-kind-int 4))
  )
  (bind ?goal-msg (cx-pddl-interfaces-plan-goal-create))
  (cx-pddl-interfaces-plan-goal-set-field ?goal-msg "pddl_instance" ?instance)
  (cx-pddl-interfaces-plan-goal-set-field ?goal-msg "goal_instance" ?goal)
  (cx-pddl-interfaces-plan-goal-set-field ?goal-msg "plan_kind" ?plan-kind-int)
  (cx-pddl-interfaces-plan-goal-set-field ?goal-msg "output_dir" ?dir)
  (cx-pddl-interfaces-plan-send-goal ?goal-msg ?server)
  (modify ?pp (state WAITING) (goal-ptr ?goal-msg))
)

(defrule pddl-plan-get-goal-response
" Process the first goal response"

  (pddl-manager (node ?node))
  (cx-pddl-interfaces-plan-client (server ?server&:(eq ?server (str-cat ?node "/plan"))))
  ?response <- (cx-pddl-interfaces-plan-goal-response (server ?server) (client-goal-handle-ptr ?gh-ptr))
  ?pp <- (pddl-plan (instance ?instance) (goal ?goal) (goal-ptr ?goal-ptr) (state WAITING))
  =>
  (bind ?status (cx-pddl-interfaces-plan-client-goal-handle-get-status ?gh-ptr))
  ; ACCEPTED, EXECUTING,  SUCCEEDED
  (if (or (= ?status 1)  (= ?status 2) (= ?status 4)) then
    (modify ?pp (state PLANNING) (goal-handle ?gh-ptr))
    (retract ?response)
    else
    ; CANCELING, CANCELED, ABORTED
    (if (or (= ?status 3) (= ?status 5) (= ?status 6))
      then
      (modify ?pp (state FAILURE) (goal-handle ?gh-ptr))
      else
      (modify ?pp (state FAILURE) (goal-handle ?gh-ptr))
      (printout error "pddl-plan: Unexpected status "  ?status crlf)
    )
  )
  (retract ?response)
)

(defrule pddl-plan-cancel-start
" Cancel a plan action"
  (pddl-manager (node ?node))
  (cx-pddl-interfaces-plan-client (server ?server&:(eq ?server (str-cat ?node "/plan"))))
  ?pp <- (pddl-plan (instance ?instance) (goal ?goal) (state REQUEST-CANCELING) (goal-handle ?gh-ptr))
=>
  (cx-pddl-interfaces-plan-client-cancel-goal ?server ?gh-ptr)
  (modify ?pp (state CANCELING))
)

(defrule pddl-plan-cancel-done
" Cancelation of plan action successful"
  (pddl-manager (node ?node))
  (pddl-instance (state LOADED) (name ?instance) (busy-with FALSE))
  (cx-pddl-interfaces-plan-client (server ?server&:(eq ?server (str-cat ?node "/plan"))))
  ?response <- (cx-pddl-interfaces-plan-cancel-goal-response (server ?server) (num-goals 1) (return-code ERROR_NONE|ERROR_GOAL_TERMINATED))
  ?entry <- (cx-pddl-interfaces-plan-cancel-goal-entry (server ?server) (goal-id ?uuid))
  ?pp <- (pddl-plan (instance ?instance) (goal ?goal) (state CANCELING) (goal-handle ?gh-ptr))
  (test (eq (cx-pddl-interfaces-plan-client-goal-handle-get-goal-id ?gh-ptr) ?uuid))
=>
  (modify ?pp (state CANCELED))
  (retract ?response ?entry)
  (cx-pddl-interfaces-plan-client-goal-handle-destroy ?gh-ptr)
)

(defrule pddl-plan-plan-result
" Retrieve the resulting plan "
  (pddl-manager (node ?node))
  (pddl-instance (name ?instance) (state LOADED) (busy-with FALSE))
  ?pp <- (pddl-plan (instance ?instance) (id ?plan-id) (goal ?goal) (goal-ptr ?goal-ptr) (plan-type ?plan-type) (state PLANNING) (goal-handle ?gh-ptr))
  ?wr-f <- (cx-pddl-interfaces-plan-wrapped-result
    (server ?server&:(eq ?server (str-cat ?node "/plan"))) (goal-id ?uuid) (code SUCCEEDED) (result-ptr ?res-ptr))
  (test (eq (cx-pddl-interfaces-plan-client-goal-handle-get-goal-id ?gh-ptr) ?uuid))
  =>
  (bind ?plan-found (cx-pddl-interfaces-plan-result-get-field ?res-ptr "success"))
  (bind ?id 0)
  (if ?plan-found then
    (bind ?plan (cx-pddl-interfaces-plan-result-get-field ?res-ptr "actions"))
    (bind ?kind (cx-pddl-interfaces-plan-result-get-field ?res-ptr "flat_plan_kind"))
    (foreach ?action ?plan
      (bind ?name (sym-cat (cx-pddl-interfaces-plan-action-get-field ?action "name")))
      (bind ?args (cx-pddl-interfaces-plan-action-get-field ?action "args"))
      (bind ?order (cx-pddl-interfaces-plan-action-get-field ?action "order"))
      (if (or (eq ?plan-type PARTIAL-ORDER) (eq ?plan-type STN))  then
        (bind ?order (cx-pddl-interfaces-plan-action-get-field ?action "action_id"))
       else
        (bind ?order (cx-pddl-interfaces-plan-action-get-field ?action "order"))
      )
      (bind ?task-id (cx-pddl-interfaces-plan-action-get-field ?action "task_id"))
      (bind ?ps-time (cx-pddl-interfaces-plan-action-get-field ?action "start_time"))
      (bind ?p-duration (cx-pddl-interfaces-plan-action-get-field ?action "duration"))
      (bind ?predecessors (cx-pddl-interfaces-plan-action-get-field ?action "predecessors"))
      (assert (pddl-action
        (id (sym-cat ?id - (gensym*)))
        (plan ?plan-id)
        (instance ?instance)
        (name ?name)
        (params ?args)
        (order ?order)
        (task-id ?task-id)
        (planned-start-time ?ps-time)
        (planned-duration ?p-duration)
        (predecessors ?predecessors))
      )
      (bind ?id (+ ?id 1))
    )
    (bind ?id 0)
    (bind ?method-plan (cx-pddl-interfaces-plan-result-get-field ?res-ptr "methods"))
    (foreach ?method ?method-plan
        (bind ?name (sym-cat (cx-pddl-interfaces-hierarchical-plan-method-get-field ?method "name")))
        (bind ?args (cx-pddl-interfaces-hierarchical-plan-method-get-field ?method "args"))
        (bind ?task-id (cx-pddl-interfaces-hierarchical-plan-method-get-field ?method "task_id"))
        (assert (pddl-method
          (id (sym-cat ?id - (gensym*)))
          (plan ?plan-id)
          (instance ?instance)
          (name ?name)
          (params ?args)
          (task-id ?task-id)
        )
      )
      (bind ?id (+ ?id 1))
    )
    (bind ?id 0)
    (bind ?stn-constraints (cx-pddl-interfaces-plan-result-get-field ?res-ptr "stn_constraints"))
    (foreach ?constraint ?stn-constraints
        (bind ?from (cx-pddl-interfaces-stn-constraint-get-field ?constraint "from_action_id"))
        (bind ?from-role (sym-cat (cx-pddl-interfaces-stn-constraint-get-field ?constraint "from_role")))
        (bind ?to (cx-pddl-interfaces-stn-constraint-get-field ?constraint "to_action_id"))
        (bind ?to-role (sym-cat (cx-pddl-interfaces-stn-constraint-get-field ?constraint "to_role")))
        (bind ?lower (cx-pddl-interfaces-stn-constraint-get-field ?constraint "lower_bound"))
        (bind ?upper (cx-pddl-interfaces-stn-constraint-get-field ?constraint "upper_bound"))
        (bind ?is-lower (cx-pddl-interfaces-stn-constraint-get-field ?constraint "is_lower_bounded"))
        (bind ?is-upper (cx-pddl-interfaces-stn-constraint-get-field ?constraint "is_upper_bounded"))
        (assert (pddl-stn-constraint
          (id (sym-cat ?id - (gensym*)))
          (plan ?plan-id)
          (instance ?instance)
          (from ?from)
          (from-role ?from-role)
          (to ?to)
          (to-role ?to-role)
          (is-lower-bounded ?is-lower)
          (is-upper-bounded ?is-upper)
          (lower-bound ?lower)
          (upper-bound ?upper)
        )
      )
      (bind ?id (+ ?id 1))
    )
    (bind ?action-type ?plan-type)
    (if (eq ?plan-type PARTIAL-ORDER) then
      (bind ?action-type CLASSICAL)
    )
    (if (eq ?plan-type HIERARCHICAL)
      then
      (if (eq ?kind 0) then
        (bind ?action-type CLASSICAL)
      else
        (bind ?action-type TEMPORAL)
      )
    )
    (modify ?pp (state SUCCESS) (action-type ?action-type))
  else
    (modify ?pp (state FAILURE))
    (printout red "plan not found!" crlf)
  )
  (retract ?wr-f)
  (cx-pddl-interfaces-plan-result-destroy ?res-ptr)
  (cx-pddl-interfaces-plan-goal-destroy ?goal-ptr)
  (cx-pddl-interfaces-plan-client-goal-handle-destroy ?gh-ptr)
)
