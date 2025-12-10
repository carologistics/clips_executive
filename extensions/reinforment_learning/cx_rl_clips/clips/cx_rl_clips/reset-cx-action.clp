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

(defrule cx-rl-reset-cx-action-server-init
  "Create an action server resetting the clips environment"
  (cx-rl-node (name ?node-name))
  (not (cx-rl-interfaces-reset-cx-server (name ?name&:(eq ?name (str-cat ?node-name "/reset_cx")))))
  (not (executive-finalize))
=>
  (bind ?name (str-cat ?node-name "/reset_cx"))
  (cx-rl-interfaces-reset-cx-create-server ?name)
  (printout ?*CX-RL-LOG-LEVEL* "Created server for " ?name crlf)
)

(deffunction cx-rl-interfaces-reset-cx-handle-goal-callback (?server ?goal ?uuid)
  (printout ?*CX-RL-LOG-LEVEL* ?server " callback (goal " ?goal " ; id " ?uuid " )" crlf)
  (return 2)
)

(deffunction cx-rl-interfaces-reset-cx-cancel-goal-callback (?server ?goal ?goal-handle)
  (return 1)
)

(defrule cx-rl-reset-cx-goal-accepted-start
  (cx-rl-node (name ?node))
  (cx-rl-interfaces-reset-cx-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  (test (str-index ?node ?server))
  (not (rl-reset-env (node ?node) (uuid ?uuid&:(eq ?uuid (cx-rl-interfaces-reset-cx-server-goal-handle-get-goal-id ?ptr)))))
=>
  (if (not (cx-rl-interfaces-reset-cx-server-goal-handle-is-canceling ?ptr)) then
      (bind ?uuid (cx-rl-interfaces-reset-cx-server-goal-handle-get-goal-id ?ptr))
      (assert (rl-reset-env (node ?node) (uuid ?uuid) (state ABORT-RUNNING-ACTIONS)))
  else
      (printout ?*CX-RL-LOG-LEVEL* "Goal immediately canceled" crlf)
  )
)


(defrule cx-rl-reset-cx-finished
  (cx-rl-node (name ?node))
  ?ag <- (cx-rl-interfaces-reset-cx-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  (cx-rl-interfaces-reset-cx-server (name ?name&:(eq ?name (str-cat ?node "/reset_cx"))))
  ?rf <- (rl-reset-env (node ?node) (state DONE))
=>
  (printout ?*CX-RL-LOG-LEVEL* "reset-game finished" crlf)
  (bind ?result (cx-rl-interfaces-reset-cx-result-create))
  (cx-rl-interfaces-reset-cx-result-set-field ?result "confirmation" "Reset completed")
  (cx-rl-interfaces-reset-cx-server-goal-handle-succeed ?ptr ?result)
  (cx-rl-interfaces-reset-cx-result-destroy ?result)
  (cx-rl-interfaces-reset-cx-server-goal-handle-destroy ?ptr)
  (retract ?rf)
  (retract ?ag)
  (assert (rl-current-action-space (node ?node) (state PENDING)))
)

(defrule cx-rl-reset-cx-server-cleanup
    (executive-finalize)
    (cx-rl-interfaces-reset-cx-server (name ?server))
=>
    (cx-rl-interfaces-reset-cx-destroy-server ?server)
)

(defrule cx-rl-reset-cx-accepted-goal-cleanup
    (executive-finalize)
    ?ag <- (cx-rl-interfaces-reset-cx-accepted-goal (server-goal-handle-ptr ?ptr))
=>
    (cx-rl-interfaces-reset-cx-server-goal-handle-destroy ?ptr)
    (retract ?ag)
)
