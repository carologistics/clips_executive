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

(defrule reset-cx-action-server-init
    "Create an action server resetting the clips environment"
    (not (cx-rl-interfaces-reset-cx-server (name "reset_cx")))
    (not (executive-finalize))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-reset-cx-create-server "reset_cx")
    (printout info "Created server for /reset_cx" crlf)
)

(deffunction cx-rl-interfaces-reset-cx-handle-goal-callback (?server ?goal ?uuid)
    (printout blue ?server " callback (goal " ?goal " ; id " ?uuid " )" crlf)
    (return 2)
)

(deffunction cx-rl-interfaces-reset-cx-cancel-goal-callback (?server ?goal ?goal-handle)
    (return 1)
)

(defrule reset-cx-goal-accepted-start
    (cx-rl-interfaces-reset-cx-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
    (not (reset-cx-action (uuid ?uuid&:(eq ?uuid (cx-rl-interfaces-reset-cx-server-goal-handle-get-goal-id ?ptr)))))
    (not (reset-cx-finished))
=>
    (if (not (cx-rl-interfaces-reset-cx-server-goal-handle-is-canceling ?ptr)) then
        (bind ?uuid (cx-rl-interfaces-reset-cx-server-goal-handle-get-goal-id ?ptr))
        (assert (reset-cx-action (uuid ?uuid)))
        (assert (reset-cx (stage INIT)))
        (assert (abort-all-action-selections))
        (assert (abort-get-free-robot))
    else
        (printout error "Goal immediately canceled" crlf)
    )
)


(defrule reset-cx-finished
    ?ag <- (cx-rl-interfaces-reset-cx-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
    ?rf <- (reset-cx-finished)
    (cx-rl-interfaces-reset-cx-server (name "reset_cx"))

=>
    (printout green "reset-cx finished" crlf)
    (bind ?result (cx-rl-interfaces-reset-cx-result-create))
    (cx-rl-interfaces-reset-cx-result-set-field ?result "confirmation" "Reset completed")
    (cx-rl-interfaces-reset-cx-server-goal-handle-succeed ?ptr ?result)
    (cx-rl-interfaces-reset-cx-result-destroy ?result)
    (cx-rl-interfaces-reset-cx-server-goal-handle-destroy ?ptr)
    (retract ?rf)
    (retract ?ag)
)

(defrule reset-cx-server-cleanup
    (executive-finalize)
    (cx-rl-interfaces-reset-cx-server (name ?server))
=>
    (cx-rl-interfaces-reset-cx-destroy-server ?server)
)

(defrule reset-cx-accepted-goal-cleanup
    (executive-finalize)
    ?ag <- (cx-rl-interfaces-reset-cx-accepted-goal (server-goal-handle-ptr ?ptr))
=>
    (cx-rl-interfaces-reset-cx-server-goal-handle-destroy ?ptr)
    (retract ?ag)
)
