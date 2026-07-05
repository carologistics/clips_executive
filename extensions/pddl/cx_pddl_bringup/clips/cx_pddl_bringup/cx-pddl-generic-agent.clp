
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

; ---------------- SETUP INSTANCE ------------------

;(defrule cx-pddl-clips-agent-pddl-init
;=>
;  (assert (pddl-manager (node "/pddl_manager")))
;)

(defrule cx-pddl-clips-agent-pddl-add-instance-advanced-production
  (test (eq (ros-param-get-value "game.agent" "production") "production"))
  =>
  (assert
    ;PLAN!
    (pddl-plan (id game3-production) (plan-type CLASSICAL) (plan-start 0.0) (state SUCCESS) (action-type CLASSICAL))
    
    ;bring part-1 to wenbo
    (pddl-action (id (gensym*)) (plan game3-production) (state IDLE) (order 0) (name pick) (params SS1-LEFT red2x2))
    (pddl-action (id (gensym*)) (plan game3-production) (state IDLE) (order 1) (name put) (params WB1 red2x2))
    ;bring part-8 to wenbo
    (pddl-action (id (gensym*)) (plan game3-production) (state IDLE) (order 2) (name pick) (params HS-LEFT yellow4x2))
    (pddl-action (id (gensym*)) (plan game3-production) (state IDLE) (order 3) (name put) (params WB1 yellow4x2))
    ;bring part-7 to wenbo
    (pddl-action (id (gensym*)) (plan game3-production) (state IDLE) (order 4) (name pick) (params SS1-RIGHT blue4x2))
    (pddl-action (id (gensym*)) (plan game3-production) (state IDLE) (order 5) (name put) (params WB1 blue4x2))
    ;bring part-5 to wenbo
    (pddl-action (id (gensym*)) (plan game3-production) (state IDLE) (order 6) (name pick) (params HS-RIGHT red4x2))
    (pddl-action (id (gensym*)) (plan game3-production) (state IDLE) (order 7) (name put) (params WB1 red4x2))


  )
)

(defrule cx-pddl-clips-agent-pddl-add-instance-advanced-delivery
  (test (eq (ros-param-get-value "game.agent" "delivery") "delivery"))
  =>
  (assert
    (pddl-action (id (gensym*)) (plan game3-delivery) (state IDLE) (order 5) (name pick) (params SS1-LEFT hammer))
    (pddl-action (id (gensym*)) (plan game3-delivery) (state IDLE) (order 9) (name put) (params CC estop))
    (pddl-plan (id game3-delivery) (plan-type CLASSICAL) (plan-start 0.0) (state SUCCESS) (action-type CLASSICAL))
    ;take part 4 back
    (pddl-action (id (gensym*)) (plan game3-delivery) (state IDLE) (order 0) (name pick) (params WB1-LEFT yellow2x2))
    (pddl-action (id (gensym*)) (plan game3-delivery) (state IDLE) (order 1) (name put) (params SS2-RIGHT yellow2x2))
    ;take part 2 back
    (pddl-action (id (gensym*)) (plan game3-delivery) (state IDLE) (order 2) (name pick) (params WB1-LEFT green2x2))
    (pddl-action (id (gensym*)) (plan game3-delivery) (state IDLE) (order 3) (name put) (params SS2-LEFT green2x2))
    ;take part 1 back
    (pddl-action (id (gensym*)) (plan game3-delivery) (state IDLE) (order 4) (name pick) (params WB1-LEFT red2x2))
    (pddl-action (id (gensym*)) (plan game3-delivery) (state IDLE) (order 5) (name put) (params SS1-LEFT red2x2))

    ;deliver hammer
    (pddl-action (id (gensym*)) (plan game3-delivery) (state IDLE) (order 6) (name pick) (params WB1-LEFT hammer))
    (pddl-action (id (gensym*)) (plan game3-delivery) (state IDLE) (order 7) (name put) (params CC hammer))
    (pddl-action (id (gensym*)) (plan game3-delivery) (state IDLE) (order 8) (name pick) (params WB1-LEFT estop))
    (pddl-action (id (gensym*)) (plan game3-delivery) (state IDLE) (order 9) (name put) (params CC estop))
    (pddl-action (id (gensym*)) (plan game3-delivery) (state IDLE) (order 10) (name pick) (params WB1-LEFT burger))
    (pddl-action (id (gensym*)) (plan game3-delivery) (state IDLE) (order 11) (name put) (params CC burger))

    ;take part 6 back
    (pddl-action (id (gensym*)) (plan game3-delivery) (state IDLE) (order 12) (name pick) (params WB1-LEFT green4x2))
    (pddl-action (id (gensym*)) (plan game3-delivery) (state IDLE) (order 13) (name put) (params SS2-LEFT green4x2))
    ;take part 3 back
    (pddl-action (id (gensym*)) (plan game3-delivery) (state IDLE) (order 14) (name pick) (params WB1-LEFT blue2x2))
    (pddl-action (id (gensym*)) (plan game3-delivery) (state IDLE) (order 15) (name put) (params HS-LEFT blue2x2))
    ;take part 8 back
    (pddl-action (id (gensym*)) (plan game3-delivery) (state IDLE) (order 16) (name pick) (params WB1-LEFT yellow4x2))
    (pddl-action (id (gensym*)) (plan game3-delivery) (state IDLE) (order 17) (name put) (params HS-LEFT yellow4x2))
    
    
  )
)

(defrule cx-pddl-clips-agent-select-action-sequential
" Start executing the first action of the resulting plan "
  ?plan <- (pddl-plan (id ?plan-id) (plan-type CLASSICAL) (plan-start ?p-start) (state SUCCESS) (action-type CLASSICAL))
  (not (pddl-action (state EXECUTING|SELECTED)))
  ?pa <- (pddl-action (plan ?plan-id) (order ?o) (state IDLE))
  (not (pddl-action (plan ?plan-id) (state IDLE) (order ?oo&:(< ?oo ?o))))
=>
  (if (= ?p-start 0.0) then (modify ?plan (plan-start (now))))
  (modify ?pa (state SELECTED))
)

(defrule cx-pddl-clips-agent-select-action-hierarchical-sequential
" Start executing the first action of the resulting plan "
  ?plan <- (pddl-plan (id ?plan-id) (plan-type HIERARCHICAL) (plan-start ?p-start) (state SUCCESS) (action-type CLASSICAL))
  (not (pddl-action (state EXECUTING|SELECTED)))
  ?pa <- (pddl-action (plan ?plan-id) (order ?o) (state IDLE))
  (not (pddl-action (plan ?plan-id) (state IDLE) (order ?oo&:(< ?oo ?o))))
=>
  (if (= ?p-start 0.0) then (modify ?plan (plan-start (now))))
  (modify ?pa (state SELECTED))
)

(defrule cx-pddl-clips-agent-select-action-hierarchical-temporal
" Start executing the first action of the resulting plan based on start time"
  ?plan <- (pddl-plan (id ?plan-id) (plan-type HIERARCHICAL) (plan-start ?p-start) (state SUCCESS) (action-type TEMPORAL))
  (not (pddl-action (state EXECUTING|SELECTED)))
  ?pa <- (pddl-action (plan ?plan-id) (planned-start-time ?t) (state IDLE))
  (not (pddl-action (plan ?plan-id) (state IDLE) (planned-start-time ?ot&:(< ?ot ?t))))
=>
  (if (= ?p-start 0.0) then (modify ?plan (plan-start (now))))
  (modify ?pa (state SELECTED))
)


(defrule cx-pddl-clips-agent-select-action-temporal
" Start executing the first action of the resulting plan based on start time"
  ?plan <- (pddl-plan (id ?plan-id) (plan-type TEMPORAL) (plan-start ?p-start) (state SUCCESS) (action-type TEMPORAL))
  (not (pddl-action (state EXECUTING|SELECTED)))
  ?pa <- (pddl-action (plan ?plan-id) (planned-start-time ?t) (state IDLE))
  (not (pddl-action (plan ?plan-id) (state IDLE) (planned-start-time ?ot&:(< ?ot ?t))))
=>
  (if (= ?p-start 0.0) then (modify ?plan (plan-start (now))))
  (modify ?pa (state SELECTED))
)

(defrule cx-pddl-clips-agent-select-action-partial-order
" Start executing the first action of the resulting plan based on order"
  ?plan <- (pddl-plan (id ?plan-id) (plan-type PARTIAL-ORDER) (plan-start ?p-start) (state SUCCESS) (action-type CLASSICAL))
  (not (pddl-action (state EXECUTING|SELECTED)))
  ?pa <- (pddl-action (plan ?plan-id) (order ?own) (predecessors) (planned-start-time ?t) (state IDLE))
=>
  (if (= ?p-start 0.0) then (modify ?plan (plan-start (now))))
  (modify ?pa (state SELECTED))
)

;(defrule cx-pddl-clips-agent-check-action
;" Before executing an action check the condition to make sure it is feasible "
;  (pddl-action (id ?id) (state SELECTED) (name ?name) (params $?params))
;  (not (pddl-action-condition (action ?id)))
;=>
;  (assert (pddl-action-condition (instance test) (action ?id)))
;)

(defrule cx-pddl-clips-agent-executable-action-mock
" Condition is satisfied, go ahead with execution "
  (pddl-plan (id ?plan-id) (plan-start ?t))
  ;(pddl-action-condition (action ?action-id) (state CONDITION-SAT))
  ?pa <- (pddl-action (id ?action-id) (plan ?plan-id) (name ?name) (params $?params) (state SELECTED))
  (test (neq (ros-param-get-value "pddl.executor" "FALSE") TRUE))
=>
  (modify ?pa (state EXECUTING) (actual-start-time (- (now) ?t)))
)

(defrule cx-pddl-clips-agent-create-executor
" Condition is satisfied, go ahead with execution "
  (pddl-plan (id ?plan-id) (plan-start ?t))
  ;(pddl-action-condition (action ?action-id) (state CONDITION-SAT))
  ?pa <- (pddl-action (id ?action-id) (plan ?plan-id) (name ?name) (params $?params) (state SELECTED))
  (test (eq (ros-param-get-value "pddl.executor" "FALSE") TRUE))
=>
  (assert (pddl-action-executor (action ?action-id)))
  (modify ?pa (state EXECUTING))
)

(defrule cx-pddl-clips-agent-done-executor
" Condition is satisfied, go ahead with execution "
  (pddl-plan (id ?plan-id) (plan-start ?t))
  ?pa <- (pddl-action (id ?action-id) (plan ?plan-id) (name ?name) (params $?params) (state EXECUTING))
  ?ex <- (pddl-action-executor (action ?action-id) (state SUCCEEDED))
=>
  (modify ?pa (state DONE))
  ;(assert (pddl-action-get-effect (action ?action-id) (apply TRUE)))
)

(defrule cx-pddl-clips-agent-execution-done-mock
" After the duration has elapsed, the action is done "
  (test (neq (ros-param-get-value "pddl.executor" "FALSE") TRUE))
  (time ?now)
  (pddl-plan (id ?plan-id) (plan-start ?t))
  ?pa <- (pddl-action (id ?id) (plan ?plan-id) (state EXECUTING) (planned-duration ?d) (name ?name)
    (actual-start-time ?s&:(< (+ ?s ?d ?t) ?now)))
=>
  (bind ?duration (- (now) (+ ?s ?t)))
  (printout info "Executed action " ?name " in " ?duration " seconds" crlf)
  (modify ?pa (state DONE) (actual-duration ?duration))
  ;(assert (pddl-action-get-effect (action ?id) (apply TRUE)))
)

(defrule cx-pddl-clips-agent-relax-partial-order
  (pddl-plan (id ?plan-id) (plan-type PARTIAL-ORDER))
  (pddl-action (order ?o) (plan ?plan-id) (state DONE))
  (pddl-action (plan ?plan-id) (id ?a-id) (state IDLE) (predecessors $? ?o $?))
  (not (pddl-action-get-effect (action ?a-id)))
  =>
  (do-for-all-facts ((?pa pddl-action))
    (and
      (eq ?pa:plan ?plan-id)
      (eq ?pa:state IDLE)
      (member$ ?o ?pa:predecessors)
    )
    (bind ?pos (member$ ?o ?pa:predecessors))
    (modify ?pa (predecessors (delete$ ?pa:predecessors ?pos ?pos)))
  )
)

(defrule cx-pddl-clips-agent-rm-get-effect-on-done
  ?f <- (pddl-action-get-effect (state DONE))
  =>
  (retract ?f)
)

(defrule cx-pddl-clips-agent-print-exec-times
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

(defrule cx-pddl-clips-agent-detect-STN-no-execution-available
" For STN plans, there is no execution available "
  ?plan <- (pddl-plan (id ?plan-id) (plan-type STN) (plan-start ?p-start) (state SUCCESS) (action-type STN))
=>
  (printout magenta "STN plan detected, no method for execution available" crlf)
  (printout magenta "Actions:" crlf)
  (do-for-all-facts ((?pa pddl-action)) TRUE
     (printout green "action " ?pa:name " "
       ?pa:params " (STN id: " ?pa:order ")" crlf
     )
  )
  (printout magenta "Constraints:" crlf)
  (do-for-all-facts ((?stn-c pddl-stn-constraint)) TRUE
     (if (and ?stn-c:is-lower-bounded ?stn-c:is-upper-bounded) then
       (printout green "action " ?stn-c:from " " ?stn-c:from-role " --[ " ?stn-c:lower-bound ", " ?stn-c:upper-bound " ]--> " ?stn-c:to " " ?stn-c:to-role crlf)
     )
     (if (and ?stn-c:is-lower-bounded (not ?stn-c:is-upper-bounded)) then
       (printout green "action " ?stn-c:from " " ?stn-c:from-role " --[ " ?stn-c:lower-bound ", INF ]--> " ?stn-c:to " " ?stn-c:to-role crlf)
     )
     (if (and (not ?stn-c:is-lower-bounded) ?stn-c:is-upper-bounded) then
       (printout green "action " ?stn-c:from " " ?stn-c:from-role " --[ -INF, " ?stn-c:upper-bound " ]--> " ?stn-c:to " " ?stn-c:to-role crlf)
     )
     (if (and (not ?stn-c:is-lower-bounded) (not ?stn-c:is-upper-bounded)) then
       (printout green "action " ?stn-c:from " " ?stn-c:from-role " --[ -INF, INF ]--> " ?stn-c:to " " ?stn-c:to-role crlf)
     )
  )
)
