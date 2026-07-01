; Copyright (c) 2024-2026 Carologistics
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

; This file showcases intefaces with ROS action servers and clients

(defglobal
  ?*ROBOTINO-BTS-ACTION-SERVER* = "/robotinobase1/robotino_behavior_server"
)

(deffunction action-to-goal-str (?action-name $?params)
 	(bind ?goal-str (str-cat "(" ?action-name))
 	(foreach ?p $?params
 		(bind ?goal-str (str-cat ?goal-str " " ?p))
 	)
 	(return (str-cat ?goal-str ")"))
)

(deffunction assert-pddl-fluent-change-from-fluent-delta ($?fluent-delta)
  (foreach ?f $?fluent-delta
    (bind ?action-list (explode$ (sub-string 2 (- (str-index ")" ?f) 1) ?f)))
    (bind ?add-str (sub-string (+ 1 (str-index "=" ?f)) (str-length ?f) ?f))
    (if (eq ?add-str "True") then (bind ?add TRUE) else (bind ?add FALSE))
    (assert (pddl-fluent-change (instance test) (name (nth$ 1 ?action-list))
        (params (rest$ ?action-list)) (delete ?add)
    ))
  )
)

(defrule tampanda-simulator-action-client-init
" Create a simple client using the generated bindings. "
  (not (tampanda-ros-interfaces-execute-pddl-action-client (server ?server&:(eq ?server ?*ROBOTINO-BTS-ACTION-SERVER*))))
  (not (executive-finalize))
=>
  (tampanda-ros-interfaces-execute-pddl-action-create-client ?*ROBOTINO-BTS-ACTION-SERVER*)
  (printout green "Created client for /execute_pddl_action" crlf)
)

(defrule tampanda-simulator-client-send-goal
  (tampanda-ros-interfaces-execute-pddl-action-client (server ?server))
  ?ex <- (pddl-action-executor (action ?id) (state INIT))
  (pddl-action (id ?id) (name ?action-name) (params $?params))
  =>
  (bind ?goal-string (action-to-goal-str ?action-name $?params))
  (bind ?goal (tampanda-ros-interfaces-execute-pddl-action-goal-create))
  (assert (tampanda-simulator-goal ?goal))
  (tampanda-ros-interfaces-execute-pddl-action-goal-set-field ?goal "pddl_action" ?goal-string)
  (tampanda-ros-interfaces-execute-pddl-action-send-goal ?goal ?server)
  (printout green "Sending goal tampanda-simulator" crlf)
  (modify ?ex (state REQUESTED) (goal-ptr ?goal))
  ; do not destroy the goal here, only do it once the goal is fully processed and finished
  ; (tampanda-ros-interfaces-execute-pddl-action-goal-destroy ?goal)
)

(defrule tampanda-simulator-get-goal-response
" Process the first goal response"
  (tampanda-ros-interfaces-execute-pddl-action-client (server ?server&:(eq ?server ?*TAMPANDA-ACTION-SERVICE*)))
  ?response <- (tampanda-ros-interfaces-execute-pddl-action-goal-response (server ?server) (client-goal-handle-ptr ?gh-ptr))
  ?ex <- (pddl-action-executor (goal-ptr ?goal-ptr) (state REQUESTED))
  =>
  (bind ?status (tampanda-ros-interfaces-execute-pddl-action-client-goal-handle-get-status ?gh-ptr))
  ; ACCEPTED, EXECUTING,  SUCCEEDED
  (if (or (= ?status 1)  (= ?status 2) (= ?status 4)) then
    (modify ?ex (state ACCEPTED) (goal-handle ?gh-ptr))
    else
    ;SUCCEEDED
    (if (= ?status 4) then
      (modify ?ex (state SUCCEEDED) (goal-handle ?gh-ptr))
     else
      ; CANCELING, CANCELED, ABORTED
      (if (or (= ?status 3) (= ?status 5) (= ?status 6))
        then
        (modify ?ex (state ABORTED) (goal-handle ?gh-ptr))
        else
        (modify ?ex (state ABORTED) (goal-handle ?gh-ptr))
        (printout error "pddl-action-executor: Unexpected status "  ?status crlf)
      )
    )
  )
  (retract ?response)
)

;(defrule tampanda-simulator-client-get-feedback
;" Print any partial compuation result received so far and perpare cancelation once the received sequence is of length 7. "
;  (declare (salience 100))
;  ?f <- (tampanda-ros-interfaces-execute-pddl-action-goal-feedback (server ?server) (client-goal-handle-ptr ?ghp) (feedback-ptr ?fp))
;  =>
;  (bind ?g-id (tampanda-ros-interfaces-execute-pddl-action-client-goal-handle-get-goal-id ?ghp))
;  (bind ?g-stamp (tampanda-ros-interfaces-execute-pddl-action-client-goal-handle-get-goal-stamp ?ghp))
;  (bind ?g-status (tampanda-ros-interfaces-execute-pddl-action-client-goal-handle-get-status ?ghp))
;  (bind ?g-is-f-aware (tampanda-ros-interfaces-execute-pddl-action-client-goal-handle-is-feedback-aware ?ghp))
;  (bind ?g-is-r-aware (tampanda-ros-interfaces-execute-pddl-action-client-goal-handle-is-result-aware ?ghp))
;  ; the stamp seems to be broken (looks like a rclcpp_action issue)
;  (printout debug "[" (- (now) ?g-stamp) "] " ?g-status " " ?g-id " f " ?g-is-f-aware " r " ?g-is-r-aware crlf)
;  (bind ?part-seq (tampanda-ros-interfaces-execute-pddl-action-feedback-get-field ?fp "sequence"))
;  (printout green "partial sequence: " ?part-seq   crlf)
;  (if (= (length$ ?part-seq) 7) then
;     (assert (cancel-goal))
;  )
;  (tampanda-ros-interfaces-execute-pddl-action-feedback-destroy ?fp)
;  (retract ?f)
;)

(defrule tampanda-simulator-client-cleanup-after-wrapped-result
  ;(declare (salience 10))
  ?ex <- (pddl-action-executor (goal-handle ?ghp) (action ?action-id) (goal-ptr ?goal-msg))
  (pddl-action (id ?action-id) (name ?action-name) (params $?action-params))
  ;?f <- (tampanda-ros-interfaces-execute-pddl-action-goal-response (server ?server) (client-goal-handle-ptr ?ghp))
  ?g <- (tampanda-ros-interfaces-execute-pddl-action-wrapped-result (server ?server) (goal-id ?uuid) (code SUCCEEDED) (result-ptr ?rp))
  (test (eq ?uuid (tampanda-ros-interfaces-execute-pddl-action-client-goal-handle-get-goal-id ?ghp)))
  (time ?now)
  =>
  (bind ?success (cx-pddl-interfaces-plan-result-get-field ?rp "success"))
  (if ?success then
    (modify ?ex (state SUCCEEDED))
    (bind $?fluent-delta (tampanda-ros-interfaces-execute-pddl-action-result-get-field ?rp "fluent_delta"))
    (printout green "received fluent delta for action " ?action-name " is " ?fluent-delta crlf)
    ;(assert-pddl-fluent-change-from-fluent-delta ?fluent-delta)
    (tampanda-ros-interfaces-execute-pddl-action-result-destroy ?rp)
    (retract ?g)
    (bind ?g-id (tampanda-ros-interfaces-execute-pddl-action-client-goal-handle-get-goal-id ?ghp))
    (bind ?g-stamp (tampanda-ros-interfaces-execute-pddl-action-client-goal-handle-get-goal-stamp ?ghp))
    (bind ?g-is-f-aware (tampanda-ros-interfaces-execute-pddl-action-client-goal-handle-is-feedback-aware ?ghp))
    (bind ?g-is-r-aware (tampanda-ros-interfaces-execute-pddl-action-client-goal-handle-is-result-aware ?ghp))
    (printout debug "Final goal response [" (- (now) ?g-stamp) "] " ?uuid " " ?g-id " f " ?g-is-f-aware " r " ?g-is-r-aware crlf)
    (tampanda-ros-interfaces-execute-pddl-action-client-goal-handle-destroy ?ghp)
    ;(retract ?f)
    (tampanda-ros-interfaces-execute-pddl-action-goal-destroy ?goal-msg)
  )
)

(defrule tampanda-simulator-client-cleanup
  (executive-finalize)
  (tampanda-ros-interfaces-execute-pddl-action-client (server ?client))
  =>
  (tampanda-ros-interfaces-execute-pddl-action-destroy-client ?client)
)

(defrule tampanda-simulator-accepted-goal-cleanup
  (executive-finalize)
  ?f <- (tampanda-simulator-goal ?p)
  =>
  (tampanda-ros-interfaces-execute-pddl-action-goal-destroy ?p)
  (retract ?f)
)

(defrule cx-pddl-clips-agent-poison-plan-before-first-action
  (not (poisoned))
  (pddl-plan (id ?id) (state SUCCESS))
  (not (pddl-action (plan ?id) (state SELECTED)))
  =>
  (do-for-all-facts ((?pa pddl-action)) (eq ?pa:plan ?id)
       (modify ?pa (order (+ ?pa:order 1)))
  )
  (assert
    (pddl-action (instance test) (id (gensym*)) (name pick-cube) (params cube_4 parts__0_4) (plan ?id) (order 0) (state IDLE)) 
    (poisoned)
  )
)