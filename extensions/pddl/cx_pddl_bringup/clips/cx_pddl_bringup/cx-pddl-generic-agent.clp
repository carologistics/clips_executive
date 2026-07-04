
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
  =>
  (assert
    ;PLAN!
    (pddl-plan (id game1) (plan-type CLASSICAL) (plan-start 0.0) (state SUCCESS) (action-type CLASSICAL))

    ;move three products to wenbo for disassembly

    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 1) (name pick) (params CC traffic-light))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 2) (name put) (params WB1 traffic-light))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 3) (name disassemble-start) (params traffic-light-disassemble))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 4) (name disassemble-end) (params traffic-light-disassemble))
    ;bring traffic light part boxes to wenbo
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 5) (name pick) (params SS1-RIGHT red2x2))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 6) (name put) (params WB1 red2x2))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 7) (name pick) (params SS2-RIGHT yellow2x2))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 8) (name put) (params WB1 yellow2x2))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 9) (name pick) (params SS2-LEFT green2x2))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 10) (name put) (params WB1 green2x2))

    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 11) (name pick) (params CC small-tree))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 12) (name put) (params WB1 small-tree-disassemble))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 13) (name disassemble-start) (params small-tree-disassemble))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 14) (name disassemble-end) (params small-tree-disassemble))

    ;bring small tree part boxes to wenbo
    ;green4x2 left, rest already brought

    ;assemble e-stop
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 15) (name pick) (params HS-RIGHT yellow4x2))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 16) (name put) (params WB1 yellow4x2))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 17) (name pick) (params HS-LEFT blue2x2))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 18) (name put) (params WB1 blue2x2))

    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 19) (name pick) (params CC ice-cream))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 20) (name put) (params WB1 ice-cream))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 21) (name disassemble-start) (params ice-cream-disassemble))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 22) (name disassemble-end) (params ice-cream-disassemble))

    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 23) (name pick) (params HS-RIGHT red4x2))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 24) (name put) (params WB1 red4x2))

    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 25) (name assemble-start) (params estop-assemble))


    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 26) (name assemble-end) (params estop-assemble))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 27) (name pick) (params WB1-LEFT finished-product))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 28) (name put) (params CC finished-product))

    ;assemble hammer
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 29) (name assemble-start) (params hammer-assemble))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 30) (name assemble-end) (params hammer-assemble))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 31) (name pick) (params WB1-LEFT finished-product))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 32) (name put) (params CC finished-product))



    ;assemble burger

    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 33) (name assemble-start) (params burger-assemble))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 34) (name pick) (params SS2 green4x2))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 35) (name put) (params WB1 green4x2))


    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 36) (name assemble-end) (params burger-assemble))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 37) (name pick) (params WB1-LEFT finished-product))
    (pddl-action (id (gensym*)) (plan game1) (state IDLE) (order 38) (name put) (params CC finished-product))




    ;bring back all recycled parts

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
