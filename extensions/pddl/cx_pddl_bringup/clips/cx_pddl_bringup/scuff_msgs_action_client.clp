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
  ?*SCUFF-MSGS-ACTION-SERVER* = "/string_command"
)

(defrule scuff-action-client-init
" Create a simple client using the generated bindings. "
  (not (scuff-msgs-string-command-client))
  (not (executive-finalize))
=>
  (scuff-msgs-string-command-create-client ?*SCUFF-MSGS-ACTION-SERVER*)
  (printout green "Created client for scuff" crlf)
)

(defrule scuff-client-send-goal-assemble-start
  (scuff-msgs-string-command-client (server ?server))
  ?ex <- (pddl-action-executor (action ?id) (state INIT))
  (pddl-action (id ?id) (name assemble-start) (params ?prod $?params))
  (test (neq (ros-param-get-value "pddl.assembly" "timer") "timer"))
  =>
  (bind ?goal (scuff-msgs-string-command-goal-create))
  (assert (scuff-goal ?goal))
  (scuff-msgs-string-command-goal-set-field ?goal "request" ?prod)
  (scuff-msgs-string-command-send-goal ?goal ?server)
  (printout green "Sending goal scuff" crlf)
  (modify ?ex (state REQUESTED) (goal-ptr ?goal))
  
)

(defrule scuff-client-process-assemble-end
  (scuff-msgs-string-command-client (server ?server))
  ?ex <- (pddl-action-executor (action ?id) (state INIT))
  (pddl-action (id ?id) (name assemble-end) (params ?prod $?params))
  (test (neq (ros-param-get-value "pddl.assembly" "timer") "timer"))
  =>
  (modify ?ex (state ACCEPTED))
  
)

(defrule scuff-get-goal-response-assemble-start
" Process the first goal response"
  (scuff-msgs-string-command-client (server ?server&:(eq ?server ?*SCUFF-MSGS-ACTION-SERVER*)))
  ?response <- (scuff-msgs-string-command-goal-response (server ?server) (client-goal-handle-ptr ?gh-ptr))
  ?ex <- (pddl-action-executor (action ?id) (goal-ptr ?goal-ptr) (state REQUESTED))
  (pddl-action (id ?id) (name assemble-start) (params ?prod $?params))
  (test (neq (ros-param-get-value "pddl.assembly" "timer") "timer"))
  =>
  (bind ?status (scuff-msgs-string-command-client-goal-handle-get-status ?gh-ptr))
  ; ACCEPTED, EXECUTING,  SUCCEEDED
  (if (or (= ?status 1)  (= ?status 2) (= ?status 4)) then
    (modify ?ex (state SUCCEEDED) (goal-handle ?gh-ptr))
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

;(defrule scuff-client-get-feedback
;" Print any partial compuation result received so far and perpare cancelation once the received sequence is of length 7. "
;  (declare (salience 100))
;  ?f <- (scuff-msgs-string-command-goal-feedback (server ?server) (client-goal-handle-ptr ?ghp) (feedback-ptr ?fp))
;  =>
;  (bind ?g-id (scuff-msgs-string-command-client-goal-handle-get-goal-id ?ghp))
;  (bind ?g-stamp (scuff-msgs-string-command-client-goal-handle-get-goal-stamp ?ghp))
;  (bind ?g-status (scuff-msgs-string-command-client-goal-handle-get-status ?ghp))
;  (bind ?g-is-f-aware (scuff-msgs-string-command-client-goal-handle-is-feedback-aware ?ghp))
;  (bind ?g-is-r-aware (scuff-msgs-string-command-client-goal-handle-is-result-aware ?ghp))
;  ; the stamp seems to be broken (looks like a rclcpp_action issue)
;  (printout debug "[" (- (now) ?g-stamp) "] " ?g-status " " ?g-id " f " ?g-is-f-aware " r " ?g-is-r-aware crlf)
;  (bind ?part-seq (scuff-msgs-string-command-feedback-get-field ?fp "sequence"))
;  (printout green "partial sequence: " ?part-seq   crlf)
;  (if (= (length$ ?part-seq) 7) then
;     (assert (cancel-goal))
;  )
;  (scuff-msgs-string-command-feedback-destroy ?fp)
;  (retract ?f)
;)

(defrule scuff-client-cleanup-after-wrapped-result
  ;(declare (salience 10))
  ?ex <- (pddl-action-executor (action ?action-id) (state ACCEPTED))
  (pddl-action-executor (goal-handle ?ghp) (action ?o-action-id) (goal-ptr ?goal-msg) (state SUCCEEDED))
  (pddl-action (id ?action-id) (name assemble-end) (params $?action-params))
  (pddl-action (id ?o-action-id) (name assemble-start) (params $?action-params))
  ;?f <- (scuff-msgs-string-command-goal-response (server ?server) (client-goal-handle-ptr ?ghp))
  ?g <- (scuff-msgs-string-command-wrapped-result (server ?server) (goal-id ?uuid) (code SUCCEEDED) (result-ptr ?rp))
  (test (eq ?uuid (scuff-msgs-string-command-client-goal-handle-get-goal-id ?ghp)))
  (time ?now)
  =>
  (bind ?success (cx-pddl-interfaces-plan-result-get-field ?rp "success"))
  ;(if ?success then
    (modify ?ex (state SUCCEEDED))
    (scuff-msgs-string-command-result-destroy ?rp)
    (retract ?g)
    (bind ?g-id (scuff-msgs-string-command-client-goal-handle-get-goal-id ?ghp))
    (bind ?g-stamp (scuff-msgs-string-command-client-goal-handle-get-goal-stamp ?ghp))
    (bind ?g-is-f-aware (scuff-msgs-string-command-client-goal-handle-is-feedback-aware ?ghp))
    (bind ?g-is-r-aware (scuff-msgs-string-command-client-goal-handle-is-result-aware ?ghp))
    (printout debug "Final goal response [" (- (now) ?g-stamp) "] " ?uuid " " ?g-id " f " ?g-is-f-aware " r " ?g-is-r-aware crlf)
    (scuff-msgs-string-command-client-goal-handle-destroy ?ghp)
    ;(retract ?f)
    (scuff-msgs-string-command-goal-destroy ?goal-msg)
  ;)
)

(defrule scuff-client-cleanup
  (executive-finalize)
  (scuff-msgs-string-command-client (server ?client))
  =>
  (scuff-msgs-string-command-destroy-client ?client)
)

(defrule scuff-accepted-goal-cleanup
  (executive-finalize)
  ?f <- (scuff-goal ?p)
  =>
  (scuff-msgs-string-command-goal-destroy ?p)
  (retract ?f)
)

(defrule scuff-bypass-assembly-with-timer
  ?ex <- (pddl-action-executor (action ?id) (state INIT))
  (pddl-action (id ?id) (name assemble-start) (params ?prod $?params))
  (test (eq (ros-param-get-value "pddl.assembly" "timer") "timer"))
  (time ?now)
  =>
  (printout info "timer for assembly starts" crlf)
  (assert (assembly-timer (start ?now)))
  (modify ?ex (state ACCEPTED))
)

(defrule scuff-assembly-time-out
  ?ex <- (pddl-action-executor (action ?id) (state ACCEPTED))
  ?timer <- (assembly-timer (start ?time))
  (pddl-action (id ?id) (name assemble-start) (params ?prod $?params))
  (time ?now)
  (test (> ?now (+ ?time (ros-param-get-value "assembly-timer" 20))))
  =>
  (printout info "timeout for assembly" crlf)
  (retract ?timer)
  (modify ?ex (state SUCCEEDED))
)

(defrule scuff-bypass-assembly-end-succeed
  ?ex <- (pddl-action-executor (action ?id) (state INIT))
  (pddl-action (id ?id) (name assemble-end) (params ?prod $?params))
  ?timer <- (assembly-timer (start ?time))
  =>
  (modify ?ex (state SUCCEEDED))
)