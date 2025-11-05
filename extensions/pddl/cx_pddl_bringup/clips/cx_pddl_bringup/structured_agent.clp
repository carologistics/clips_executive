
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

(defrule structured-agent-pddl-init
" Initiate the service clients for the pddl manager "
  (not (pddl-services-loaded))
  =>
  ; create clints for all services
  (bind ?services (create$
      add_fluents AddFluents
      add_pddl_instance AddPddlInstance
      get_fluents GetFluents
      set_goals SetGoals
  check_action_condition CheckActionCondition
      create_goal_instance CreateGoalInstance
  get_action_effects GetActionEffects
      add_fluents AddFluents
      rm_fluents RemoveFluents

  ))
  (bind ?index 1)
  (bind ?length (length$ ?services))
  (while (< ?index ?length)
      (bind ?service-name (nth$ ?index ?services))
      (bind ?service-type (nth$ (+ ?index 1) ?services))
      (ros-msgs-create-client
          (str-cat "/pddl_manager" "/" ?service-name)
          (str-cat "cx_pddl_msgs/srv/" ?service-type)
      )
      (bind ?index (+ ?index 2))
  )
  (cx-pddl-msgs-plan-temporal-create-client (str-cat "/pddl_manager" "/temp_plan"))
  (assert (pddl-services-loaded))
)

; ---------------- SETUP INSTANCE ------------------

(defrule structured-agent-pddl-add-instance
" Setup PDDL instance with an active goal to plan for "
  (not (pddl-loaded))
  (pddl-services-loaded)
  =>
  (bind ?share-dir (ament-index-get-package-share-directory "cx_pddl_bringup"))
  (assert
    (pddl-manager (node "/pddl_manager"))
    (pddl-instance
      (name test)
      (domain "domain.pddl")
      (problem "problem.pddl")
      (directory (str-cat ?share-dir "/pddl"))
    )
    (pddl-get-fluents (instance test))
    (pddl-create-goal-instance (instance test) (goal active-goal))
    (pddl-goal-fluent (instance test) (goal active-goal) (name on) (params a b))
    (pddl-goal-fluent (instance test) (goal active-goal) (name on) (params b c))
    (pddl-set-goals (instance test) (goal active-goal))
  )
)

(defrule structured-agent-pddl-plan
" Start the planner once the set-goals request is done "
  (cx-pddl-msgs-plan-temporal-client (server ?server&:(eq ?server "/pddl_manager/temp_plan")))
  (not (planned))
  (pddl-set-goals (state DONE))
  =>
  (printout green "Start planning" crlf)
  (bind ?goal (cx-pddl-msgs-plan-temporal-goal-create))
  (cx-pddl-msgs-plan-temporal-goal-set-field ?goal "pddl_instance" "test")
  (cx-pddl-msgs-plan-temporal-goal-set-field ?goal "goal_instance" "active-goal")
  (cx-pddl-msgs-plan-temporal-send-goal ?goal ?server)
  (assert (planned))
)

(defrule structured-agent-pddl-plan-result
" Retrieve the resulting plan "
  ?wr-f <- (cx-pddl-msgs-plan-temporal-wrapped-result
    (server "/pddl_manager/temp_plan") (code SUCCEEDED) (result-ptr ?res-ptr)
  )
  =>
  (bind ?plan-found (cx-pddl-msgs-plan-temporal-result-get-field ?res-ptr "success"))
  (printout green "planning done" crlf)
  (bind ?id 0)
  (if ?plan-found then
    (bind ?plan (cx-pddl-msgs-plan-temporal-result-get-field ?res-ptr "actions"))
    (foreach ?action ?plan
      (bind ?name (sym-cat (cx-pddl-msgs-timed-plan-action-get-field ?action "name")))
      (bind ?args (cx-pddl-msgs-timed-plan-action-get-field ?action "args"))
      (bind ?ps-time (cx-pddl-msgs-timed-plan-action-get-field ?action "start_time"))
      (bind ?p-duration (cx-pddl-msgs-timed-plan-action-get-field ?action "duration"))
      (assert (pddl-action
        (id ?id)
        (instance test)
        (name ?name)
        (params ?args)
        (planned-start-time ?ps-time)
        (planned-duration ?p-duration))
      )
      (printout t ?ps-time "(" ?p-duration ")   " ?name ?args crlf)
      (bind ?id (+ ?id 1))
    )
  else
    (printout red "plan not found!" crlf)
  )
  (cx-pddl-msgs-plan-temporal-result-destroy ?res-ptr)
)

(defrule structured-agent-select-action
" Start executing the first action of the resulting plan "
  (not (plan-start ?t))
  ?pa <- (pddl-action (planned-start-time ?t) (state IDLE))
  (not (pddl-action (planned-start-time ?ot&:(< ?ot ?t))))
  =>
  (modify ?pa (state SELECTED))
  (assert (plan-start (now)))
)

(defrule structured-agent-check-action
" Before executing an action check the condition to make sure it is feasible "
  (pddl-action (id ?id) (state SELECTED) (name ?name) (params $?params))
  (not (pddl-action-condition (action ?id)))
  =>
  (assert (pddl-action-condition (instance test) (action ?id)))
)

(defrule structured-agent-executable-action
" Condition is satisfied, go ahead with execution "
  (plan-start ?t)
  (pddl-action-condition (action ?action-id) (state CONDITION-SAT))
  ?pa <- (pddl-action (id ?action-id) (name ?name) (params $?params) (state SELECTED))
  =>
  (modify ?pa (state EXECUTING) (actual-start-time (- (now) ?t)))
)

(defrule structured-agent-execution-done
" After the duration has elapsed, the action is done "
  (time ?now)
  (plan-start ?t)
  ?pa <- (pddl-action (id ?id) (state EXECUTING) (planned-duration ?d)
    (actual-start-time ?s&:(< (+ ?s ?d ?t) ?now)))
  =>
  (modify ?pa (state DONE) (actual-duration (- (now) (+ ?s ?t))))
  (assert (pddl-action-get-effect (action ?id) (apply TRUE)))
)

(defrule structured-agent-select-next-action
" Once an action is done, select one with lowest planned start time next "
  (not (pddl-action (state EXECUTING|SELECTED)))
  (not (pddl-action-get-effect (state ~DONE)))
  (not (pddl-fluent-change))
  ?pa <- (pddl-action (planned-start-time ?t) (state IDLE))
  (not (pddl-action (state IDLE) (planned-start-time ?ot&:(< ?ot ?t))))
  =>
  (modify ?pa (state SELECTED))
)

(defrule structured-agent-print-exec-times
" Once everything is done, print out planned vs actual times "
  (pddl-action)
  (not (pddl-action (state ~DONE)))
  (not (printed))
  =>
  (printout blue "Execution done" crlf)
  (do-for-all-facts ((?pa pddl-action)) TRUE
     (printout green "action " ?pa:name " "
       ?pa:params " " ?pa:planned-start-time "|" ?pa:planned-duration
       " vs actual " ?pa:actual-start-time "|" ?pa:actual-duration crlf
     )
  )
  (assert (printed))
)
