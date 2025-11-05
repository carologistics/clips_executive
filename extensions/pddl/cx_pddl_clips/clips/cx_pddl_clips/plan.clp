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

(deftemplate pddl-planner-call
  (slot context (type SYMBOL))
  (slot uuid (type STRING))
  (slot status (type SYMBOL) (allowed-values PENDING UNKNOWN ACCEPTED EXECUTING CANCELING SUCCEEDED CANCELED ABORTED) (default PENDING))
  (slot client-goal-handle (type EXTERNAL-ADDRESS))
  (slot goal (type EXTERNAL-ADDRESS))
)

(deffunction client-status-to-sym (?status-int)
  (if (= ?status-int 0) then (return UNKNOWN))
  (if (= ?status-int 1) then (return ACCEPTED))
  (if (= ?status-int 2) then (return EXECUTING))
  (if (= ?status-int 3) then (return CANCELING))
  (if (= ?status-int 4) then (return SUCCEEDED))
  (if (= ?status-int 5) then (return CANCELED))
  (if (= ?status-int 6) then (return ABORTED))
  (printout error "Unknown client status " ?status-int " (expected [0,6])" crlf)
  (return UNKNOWN)
)

(defrule plan-start-status-checks
  ?gr-f <- (pddl-msgs-plan-temporal-goal-response (server ?server) (client-goal-handle-ptr ?cgh-ptr))
  ?pc-f <- (pddl-planner-call (status PENDING))
  =>
  (bind ?status (pddl-msgs-plan-temporal-client-goal-handle-get-status ?cgh-ptr))
  (bind ?uuid (pddl-msgs-plan-temporal-client-goal-handle-get-goal-id ?cgh-ptr))
  (modify ?pc-f (uuid ?uuid)
    (status (client-status-to-sym ?status))
    (client-goal-handle ?cgh-ptr)
  )
  (retract ?gr-f)
)

(defrule plan-update-plan-status
  ?pc <- (pddl-planner-call (status ?status&: (member$ ?status (create$ UNKNOWN ACCEPTED EXECUTING CANCELING))) (client-goal-handle ?cgh-ptr))
  (time ?) ; poll the update
  =>
  (bind ?new-status (pddl-msgs-plan-temporal-client-goal-handle-get-status ?cgh-ptr))
  (bind ?new-status (client-status-to-sym ?new-status))
  (if (neq ?status ?new-status) then
    (modify ?pc (status ?new-status))
  )
)

(defrule plan-get-result
  ?pc-f <- (pddl-planner-call (client-goal-handle ?cgh-ptr) (goal ?goal-ptr) (uuid ?goal-id) (context ?context))
  ?wr-f <- (pddl-msgs-plan-temporal-wrapped-result (server "/pddl_manager/temp_plan") (goal-id ?goal-id) (code SUCCEEDED) (result-ptr ?res-ptr))
  =>
  (bind ?plan-found (pddl-msgs-plan-temporal-result-get-field ?res-ptr "success"))
  (printout green "planning done" crlf)
  (bind ?plan-id (gensym*))
  (bind ?instance nil)
  (if ?plan-found then
    (bind ?plan (pddl-msgs-plan-temporal-result-get-field ?res-ptr "actions"))
    (foreach ?action ?plan
      (bind ?instance (sym-cat (pddl-msgs-timed-plan-action-get-field ?action "pddl_instance")))
      (bind ?name (sym-cat (pddl-msgs-timed-plan-action-get-field ?action "name")))
      (bind ?args (pddl-msgs-timed-plan-action-get-field ?action "args"))
      (bind ?arg-syms (create$))
      (foreach ?arg ?args
        (bind ?arg-syms (create$ ?arg-syms (sym-cat ?arg)))
      )
      (bind ?equiv_class (pddl-msgs-timed-plan-action-get-field ?action "equiv_class"))
      (bind ?ps-time (pddl-msgs-timed-plan-action-get-field ?action "start_time"))
      (bind ?p-duration (pddl-msgs-timed-plan-action-get-field ?action "duration"))
      (assert (pddl-action (id (gensym*)) (plan ?plan-id) (instance ?instance) (name ?name) (params ?arg-syms)
                   (plan-order-class ?equiv_class) (planned-start-time ?ps-time) (planned-duration ?p-duration)))
      (assert (pddl-plan (id ?plan-id) (instance ?instance) (context ?context)))
    )
   else
    (printout red "plan not found!" crlf)
  )
  (pddl-msgs-plan-temporal-result-destroy ?res-ptr)
  (pddl-msgs-plan-temporal-goal-destroy ?goal-ptr)
  (pddl-msgs-plan-temporal-client-goal-handle-destroy ?cgh-ptr)
  (retract ?pc-f)
  (retract ?wr-f)
)


(defrule plan-wm-update-create-subscription
  "Create a subscription on the pddl_manager's instance update topic"
  (not (ros-msgs-subscription (topic "pddl_manager/instance_update")))
=>
  (ros-msgs-create-subscription "pddl_manager/instance_update" "std_msgs/msg/String")
  (printout info "Listening for String messages on /pddl_manager/instance_update" crlf)
)

(defrule plan-wm-update-recv
  "React to incoming messages on the pddl_manager's instance update topic"
  (ros-msgs-subscription (topic ?sub&:(eq ?sub "pddl_manager/instance_update")))
  ?msg-f <- (ros-msgs-message (topic ?sub) (msg-ptr ?inc-msg))
  =>
  (bind ?recv (ros-msgs-get-field ?inc-msg "data"))
  (delayed-do-for-all-facts ((?pi-f pddl-instance-update)) (eq ?pi-f:instance (sym-cat ?recv))
    (retract ?pi-f)
  )
  (assert (pddl-instance-update (instance (sym-cat ?recv)) (last-updated (now))))
  (ros-msgs-destroy-message ?inc-msg)
  (retract ?msg-f)
)

(defrule plan-select
  "Select a plan for execution based on its feasibility"
  ?p <- (pddl-plan (state PENDING))
  =>
  (modify ?p (state SELECTED))
)
