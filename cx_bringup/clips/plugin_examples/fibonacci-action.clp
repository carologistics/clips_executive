; Copyright (c) 2024-2025 Carologistics
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

(deffunction example-interfaces-fibonacci-handle-goal-callback (?server ?goal ?uuid)
  (printout blue ?server " callback (goal " ?goal " ; id " ?uuid  " )" crlf)
  ; (return 1) ; REJECT
  (return 2) ; ACCEPT_AND_EXECUTE
  ; (return 3) ; ACCEPT_AND_DEFER
)

(deffunction example-interfaces-fibonacci-cancel-goal-callback (?server ?goal ?goal-handle)
  ; (return 0) ; REJECT
  (return 1) ; ACCEPT
)

(defrule fibonacci-action-client-server-init
" Create a simple client and service using the generated bindings. "
  (not (example-interfaces-fibonacci-client (server "ros_cx_fibonacci")))
  (not (example-interfaces-fibonacci-server (name "ros_cx_fibonacci")))
  (not (executive-finalize))
  ;(not (client-server-already-created))
=>
  ;(assert (client-server-already-created))
  (example-interfaces-fibonacci-create-client "ros_cx_fibonacci")
  (printout info "Created client for /ros_cx_fibonacci" crlf)
  (example-interfaces-fibonacci-create-server "ros_cx_fibonacci")
  (printout info "Created server for /ros_cx_fibonacci" crlf)
)

(deftemplate fibonacci
  (slot uuid (type STRING))
  (slot order (type INTEGER))
  (slot progress (type INTEGER))
  (multislot sequence (type INTEGER))
  (slot result (type INTEGER))
  (slot last-computed (type FLOAT))
)

(defrule fibonacci-goal-accepted-start-compute
  (example-interfaces-fibonacci-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  (not (fibonacci (uuid ?uuid&:(eq ?uuid (example-interfaces-fibonacci-server-goal-handle-get-goal-id ?ptr)))))
  =>
  (if (not (example-interfaces-fibonacci-server-goal-handle-is-canceling ?ptr)) then
    (bind ?goal (example-interfaces-fibonacci-server-goal-handle-get-goal ?ptr))
    (bind ?order (example-interfaces-fibonacci-goal-get-field ?goal "order"))
    (bind ?uuid (example-interfaces-fibonacci-server-goal-handle-get-goal-id ?ptr))
    (assert (fibonacci (uuid ?uuid) (order ?order) (progress 2) (result 0) (sequence (create$ 0 1)) (last-computed (now))))
   else
    (printout error "Somehow the goal is canceling already" crlf)
  )
  ; do not destroy the server goal handle here, only do it once the goal is fully processed and finished
  ; (example-interfaces-fibonacci-server-goal-handle-destroy ?ptr)
)

(defrule fibonacci-compute-next
  (example-interfaces-fibonacci-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  ?f <- (fibonacci (order ?order) (progress ?remaining&:(>= ?order ?remaining))
  (last-computed ?computed) (result ?old-res) (sequence $?seq) (uuid ?uuid))
  (time ?now&:(> (- ?now ?computed) 1))
  (test (eq ?uuid (example-interfaces-fibonacci-server-goal-handle-get-goal-id ?ptr)))
  =>
  (bind ?step (+ ?remaining 1))
  (bind ?res (+ (nth$ ?remaining ?seq) (nth$ (- ?remaining 1) ?seq)))
  (printout magenta "Computing partial result fibonacci(" ?remaining ") = " ?res crlf)
  (bind ?seq (create$ ?seq ?res))
  (modify ?f (progress ?step) (result (+ ?old-res ?res)) (sequence ?seq))
  (bind ?feedback (example-interfaces-fibonacci-feedback-create))
  (example-interfaces-fibonacci-feedback-set-field ?feedback "sequence" ?seq)
  (example-interfaces-fibonacci-server-goal-handle-publish-feedback ?ptr ?feedback)
  (example-interfaces-fibonacci-feedback-destroy ?feedback)
  (modify ?f (last-computed ?now))
)

(defrule fibonacci-compute-done
  ?ag <- (example-interfaces-fibonacci-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  ?f <- (fibonacci (order ?order) (progress ?remaining&:(< ?order ?remaining)) (result ?old-res) (sequence $?seq) (uuid ?uuid&:(eq ?uuid (example-interfaces-fibonacci-server-goal-handle-get-goal-id ?ptr))))
  =>
  (printout green "Final fibonacci sequence (server): " ?seq crlf)
  (bind ?result (example-interfaces-fibonacci-result-create))
  (example-interfaces-fibonacci-result-set-field ?result "sequence" ?seq)
  (example-interfaces-fibonacci-server-goal-handle-succeed ?ptr ?result)
  (example-interfaces-fibonacci-result-destroy ?result)
  (example-interfaces-fibonacci-server-goal-handle-destroy ?ptr)
  (retract ?f)
  (retract ?ag)
)

(defrule fibonacci-client-send-goal
  (example-interfaces-fibonacci-client (server ?server))
  (not (send-request))
  =>
  (assert (send-request))
  (bind ?goal (example-interfaces-fibonacci-goal-create))
  (assert (fibnoacci-goal ?goal))
  (example-interfaces-fibonacci-goal-set-field ?goal "order" 5)
  (example-interfaces-fibonacci-send-goal ?goal ?server)

  ; do not destroy the goal here, only do it once the goal is fully processed and finished
  ; (example-interfaces-fibonacci-goal-destroy ?goal)
)

(defrule fibonacci-client-get-feedback
  (declare (salience 100))
  ?f <- (example-interfaces-fibonacci-goal-feedback (server ?server) (client-goal-handle-ptr ?ghp) (feedback-ptr ?fp))
  =>
  (bind ?g-id (example-interfaces-fibonacci-client-goal-handle-get-goal-id ?ghp))
  (bind ?g-stamp (example-interfaces-fibonacci-client-goal-handle-get-goal-stamp ?ghp))
  (bind ?g-status (example-interfaces-fibonacci-client-goal-handle-get-status ?ghp))
  (bind ?g-is-f-aware (example-interfaces-fibonacci-client-goal-handle-is-feedback-aware ?ghp))
  (bind ?g-is-r-aware (example-interfaces-fibonacci-client-goal-handle-is-result-aware ?ghp))
  ; the stamp seems to be broken (looks like a rclcpp_action issue)
  (printout cyan "[" (- (now) ?g-stamp) "] " ?g-status " " ?g-id " f " ?g-is-f-aware " r " ?g-is-r-aware crlf)
  (bind ?part-seq (example-interfaces-fibonacci-feedback-get-field ?fp "sequence"))
  (printout blue "partial sequence: " ?part-seq   crlf)
  (example-interfaces-fibonacci-feedback-destroy ?fp)
  (retract ?f)
)

(defrule fibonacci-client-cleanup-after-wrapped-result
  (declare (salience 10))
  ?f <- (example-interfaces-fibonacci-goal-response (server ?server) (client-goal-handle-ptr ?ghp))
  ?g <- (example-interfaces-fibonacci-wrapped-result (server ?server) (goal-id ?uuid) (code SUCCEEDED) (result-ptr ?rp))
  ?request-goal <- (fibnoacci-goal ?goal)
  (time ?now)
  =>
  (bind ?g-status (example-interfaces-fibonacci-client-goal-handle-get-status ?ghp))
  (if (> ?g-status 3) then ; status is final in one way or another
    (bind ?seq (example-interfaces-fibonacci-result-get-field ?rp "sequence"))
    (printout green "Final fibonacci sequence (client): " ?seq crlf)
    (example-interfaces-fibonacci-result-destroy ?rp)
    (retract ?g)
    (bind ?g-id (example-interfaces-fibonacci-client-goal-handle-get-goal-id ?ghp))
    (bind ?g-stamp (example-interfaces-fibonacci-client-goal-handle-get-goal-stamp ?ghp))
    (bind ?g-is-f-aware (example-interfaces-fibonacci-client-goal-handle-is-feedback-aware ?ghp))
    (bind ?g-is-r-aware (example-interfaces-fibonacci-client-goal-handle-is-result-aware ?ghp))
    (printout cyan "Final goal response [" (- (now) ?g-stamp) "] " ?uuid " " ?g-status " " ?g-id " f " ?g-is-f-aware " r " ?g-is-r-aware crlf)
    (example-interfaces-fibonacci-client-goal-handle-destroy ?ghp)
    (retract ?f)
    (example-interfaces-fibonacci-goal-destroy ?goal)
    (retract ?request-goal)
  )
)

(defrule fibonacci-client-server-cleanup
  (executive-finalize)
  (example-interfaces-fibonacci-client (server ?client))
  (example-interfaces-fibonacci-server (name ?server))
  =>
  (example-interfaces-fibonacci-destroy-client ?client)
  (example-interfaces-fibonacci-destroy-server ?server)
)

(defrule fibonacci-goal-response-cleanup
  (executive-finalize)
  ?f <- (example-interfaces-fibonacci-goal-response (client-goal-handle-ptr ?p))
  =>
  (example-interfaces-fibonacci-client-goal-handle-destroy ?p)
  (retract ?f)
)

(defrule fibonacci-accepted-goal-cleanup
  (executive-finalize)
  ?f <- (example-interfaces-fibonacci-accepted-goal (server-goal-handle-ptr ?p))
  =>
  (example-interfaces-fibonacci-server-goal-handle-destroy ?p)
  (retract ?f)
)

(defrule fibonacci-accepted-goal-cleanup
  (executive-finalize)
  ?f <- (fibnoacci-goal p)
  =>
  (example-interfaces-fibonacci--goal-destroy ?p)
  (retract ?f)
)
