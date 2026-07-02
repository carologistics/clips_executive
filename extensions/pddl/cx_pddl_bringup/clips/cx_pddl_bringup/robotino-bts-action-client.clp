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
  ?*ROBOTINO-BTS-PAYLOAD-PICK* = "{\"machine_input_tf\":location,\"reference_frame\":\"base_link\",\"object_type\":\"eurobox\",\"action\":\"pick\"}"
  ?*ROBOTINO-BTS-PAYLOAD-PUT* = "{\"machine_input_tf\":location,\"reference_frame\":\"base_link\",\"object_type\":\"eurobox\",\"action\":\"put\"}"
)

(deffunction get-payload-str (?loc ?payload)
 	(return
    (str-replace ?payload location (str-cat "\"" ?loc "\""))
  )
)

(defrule robotino-bts-action-client-init
" Create a simple client using the generated bindings. "
  (not (btcpp-ros2-interfaces-execute-tree-client (server ?server&:(eq ?server ?*ROBOTINO-BTS-ACTION-SERVER*))))
  (not (executive-finalize))
=>
  (btcpp-ros2-interfaces-execute-tree-create-client ?*ROBOTINO-BTS-ACTION-SERVER*)
  (printout green "Created client for /robotinobase1/robotino_behavior_server" crlf)
)

(defrule robotino-bts-client-send-goal-pick-put
  (btcpp-ros2-interfaces-execute-tree-client (server ?server))
  ?ex <- (pddl-action-executor (action ?id) (state INIT))
  (pddl-action (id ?id) (name ?action-name&:(or (eq ?action-name pick) (eq ?action-name put))) (params ?loc $?params))
  =>
  ;(bind ?goal-string (action-to-goal-str ?action-name $?params))
  (bind ?goal (btcpp-ros2-interfaces-execute-tree-goal-create))
  (assert (robotino-bts-goal ?goal))
  (if (eq ?action-name put) then (bind ?payload ?*ROBOTINO-BTS-PAYLOAD-PUT*)
     else
     (bind ?payload ?*ROBOTINO-BTS-PAYLOAD-PICK*)
  )
  (bind ?payload (get-payload-str ?loc ?payload))
  (printout green "calling action server with payload " ?payload crlf)
  (btcpp-ros2-interfaces-execute-tree-goal-set-field ?goal "target_tree" ManipulateObject)
  (btcpp-ros2-interfaces-execute-tree-goal-set-field ?goal "payload" ?payload)
  (btcpp-ros2-interfaces-execute-tree-send-goal ?goal ?server)
  (printout green "Sending goal robotino-bts" crlf)
  (modify ?ex (state REQUESTED) (goal-ptr ?goal))
)

(defrule robotino-bts-client-send-goal-assemble
  (btcpp-ros2-interfaces-execute-tree-client (server ?server))
  ?ex <- (pddl-action-executor (action ?id) (state INIT))
  (pddl-action (id ?id) (name assemble) (params $?params))
  =>
  (timer )
  (printout green "we are now assembling")
)

(defrule robotino-bts-get-goal-response
" Process the first goal response"
  (btcpp-ros2-interfaces-execute-tree-client (server ?server&:(eq ?server ?*ROBOTINO-BTS-ACTION-SERVER*)))
  ?response <- (btcpp-ros2-interfaces-execute-tree-goal-response (server ?server) (client-goal-handle-ptr ?gh-ptr))
  ?ex <- (pddl-action-executor (goal-ptr ?goal-ptr) (state REQUESTED))
  =>
  (bind ?status (btcpp-ros2-interfaces-execute-tree-client-goal-handle-get-status ?gh-ptr))
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

;(defrule robotino-bts-client-get-feedback
;" Print any partial compuation result received so far and perpare cancelation once the received sequence is of length 7. "
;  (declare (salience 100))
;  ?f <- (btcpp-ros2-interfaces-execute-tree-goal-feedback (server ?server) (client-goal-handle-ptr ?ghp) (feedback-ptr ?fp))
;  =>
;  (bind ?g-id (btcpp-ros2-interfaces-execute-tree-client-goal-handle-get-goal-id ?ghp))
;  (bind ?g-stamp (btcpp-ros2-interfaces-execute-tree-client-goal-handle-get-goal-stamp ?ghp))
;  (bind ?g-status (btcpp-ros2-interfaces-execute-tree-client-goal-handle-get-status ?ghp))
;  (bind ?g-is-f-aware (btcpp-ros2-interfaces-execute-tree-client-goal-handle-is-feedback-aware ?ghp))
;  (bind ?g-is-r-aware (btcpp-ros2-interfaces-execute-tree-client-goal-handle-is-result-aware ?ghp))
;  ; the stamp seems to be broken (looks like a rclcpp_action issue)
;  (printout debug "[" (- (now) ?g-stamp) "] " ?g-status " " ?g-id " f " ?g-is-f-aware " r " ?g-is-r-aware crlf)
;  (bind ?part-seq (btcpp-ros2-interfaces-execute-tree-feedback-get-field ?fp "sequence"))
;  (printout green "partial sequence: " ?part-seq   crlf)
;  (if (= (length$ ?part-seq) 7) then
;     (assert (cancel-goal))
;  )
;  (btcpp-ros2-interfaces-execute-tree-feedback-destroy ?fp)
;  (retract ?f)
;)

(defrule robotino-bts-client-cleanup-after-wrapped-result
  ;(declare (salience 10))
  ?ex <- (pddl-action-executor (goal-handle ?ghp) (action ?action-id) (goal-ptr ?goal-msg))
  (pddl-action (id ?action-id) (name ?action-name) (params $?action-params))
  ;?f <- (btcpp-ros2-interfaces-execute-tree-goal-response (server ?server) (client-goal-handle-ptr ?ghp))
  ?g <- (btcpp-ros2-interfaces-execute-tree-wrapped-result (server ?server) (goal-id ?uuid) (code SUCCEEDED) (result-ptr ?rp))
  (test (eq ?uuid (btcpp-ros2-interfaces-execute-tree-client-goal-handle-get-goal-id ?ghp)))
  (time ?now)
  =>
  (bind ?success (cx-pddl-interfaces-plan-result-get-field ?rp "success"))
  ;(if ?success then
    (modify ?ex (state SUCCEEDED))
    (btcpp-ros2-interfaces-execute-tree-result-destroy ?rp)
    (retract ?g)
    (bind ?g-id (btcpp-ros2-interfaces-execute-tree-client-goal-handle-get-goal-id ?ghp))
    (bind ?g-stamp (btcpp-ros2-interfaces-execute-tree-client-goal-handle-get-goal-stamp ?ghp))
    (bind ?g-is-f-aware (btcpp-ros2-interfaces-execute-tree-client-goal-handle-is-feedback-aware ?ghp))
    (bind ?g-is-r-aware (btcpp-ros2-interfaces-execute-tree-client-goal-handle-is-result-aware ?ghp))
    (printout debug "Final goal response [" (- (now) ?g-stamp) "] " ?uuid " " ?g-id " f " ?g-is-f-aware " r " ?g-is-r-aware crlf)
    (btcpp-ros2-interfaces-execute-tree-client-goal-handle-destroy ?ghp)
    ;(retract ?f)
    (btcpp-ros2-interfaces-execute-tree-goal-destroy ?goal-msg)
  ;)
)

(defrule robotino-bts-client-cleanup
  (executive-finalize)
  (btcpp-ros2-interfaces-execute-tree-client (server ?client))
  =>
  (btcpp-ros2-interfaces-execute-tree-destroy-client ?client)
)

(defrule robotino-bts-accepted-goal-cleanup
  (executive-finalize)
  ?f <- (robotino-bts-goal ?p)
  =>
  (btcpp-ros2-interfaces-execute-tree-goal-destroy ?p)
  (retract ?f)
)