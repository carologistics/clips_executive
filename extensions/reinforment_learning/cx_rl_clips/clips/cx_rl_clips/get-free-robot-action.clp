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

(defrule get-free-robot-action-server-init
  "Create an action server handling requests to determine the robot waiting for a goal selection"
  (cx-rl-node (name ?node-name))
  (not (cx-rl-interfaces-get-free-robot-server (name ?name&:(eq ?name (str-cat ?node-name "/get_free_robot")))))
  (not (executive-finalize))
=>
  (bind ?name (str-cat ?node-name "/get_free_robot"))
  (cx-rl-interfaces-get-free-robot-create-server ?name)
  (printout ?*CX-RL-LOG-LEVEL* "Created server for " ?name  crlf)
)

(deffunction cx-rl-interfaces-get-free-robot-handle-goal-callback (?server ?goal ?uuid)
    (return 2)
)

(deffunction cx-rl-interfaces-get-free-robot-cancel-goal-callback (?server ?goal ?goal-handle)
    (return 1)
)

(defrule get-free-robot-goal-accepted-start-search
  (cx-rl-node (name ?node))
  (cx-rl-interfaces-get-free-robot-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  (test (str-index ?node ?server))
  (not (rl-ros-action-meta-get-free-robot (node ?node) (uuid ?uuid&:(eq ?uuid (cx-rl-interfaces-get-free-robot-server-goal-handle-get-goal-id ?ptr)))))
  (time ?now)
=>
  (if (not (cx-rl-interfaces-get-free-robot-server-goal-handle-is-canceling ?ptr)) then
      (bind ?uuid (cx-rl-interfaces-get-free-robot-server-goal-handle-get-goal-id ?ptr))
      (assert (rl-ros-action-meta-get-free-robot (node ?node) (uuid ?uuid) (robot "") (last-search ?now) (found FALSE)))
  else
      (printout ?*CX-RL-LOG-LEVEL* "Goal immediately canceled" crlf)
  )
)

(defrule get-free-robot-robot-found
  (cx-rl-node (name ?node))
  (cx-rl-interfaces-get-free-robot-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  (test (str-index ?node ?server))
  ?gfr <- (rl-ros-action-meta-get-free-robot (node ?node) (uuid ?uuid) (robot "") (last-search ?last) (found FALSE) (abort-action FALSE))
  (test (eq ?uuid (cx-rl-interfaces-get-free-robot-server-goal-handle-get-goal-id ?ptr)))
  (rl-robot (node ?node) (name ?robot-name) (waiting TRUE))
  (rl-action (node ?node) (is-selected FALSE) (assigned-to ?robot-name))
  (rl-current-action-space (node ?node) (state DONE))
=>
  (modify ?gfr (robot (str-cat ?robot-name)) (last-search (now)) (found TRUE))
)

(defrule get-free-robot-episode-end
  (cx-rl-node (name ?node))
  (cx-rl-interfaces-get-free-robot-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  (test (str-index ?node ?server))
  ?gfr <- (rl-ros-action-meta-get-free-robot (node ?node) (uuid ?uuid) (robot "") (last-search ?last) (found FALSE) (abort-action FALSE))
  (test (eq ?uuid (cx-rl-interfaces-get-free-robot-server-goal-handle-get-goal-id ?ptr)))
  (rl-robot (node ?node) (name ?robot-name) (waiting TRUE))
  (rl-episode-end (node ?node) (success ?success))
  (rl-current-action-space (node ?node) (state DONE))
=>
  (modify ?gfr (last-search (now)) (found TRUE))
)

(defrule get-free-robot-search-update
  (cx-rl-node (name ?node))
  (cx-rl-interfaces-get-free-robot-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  (test (str-index ?node ?server))
  ?gfr <- (rl-ros-action-meta-get-free-robot (node ?node) (uuid ?uuid) (robot "") (last-search ?last) (found FALSE) (abort-action FALSE))
  (time ?now&:(> (- ?now ?last) ?*CX-RL-GET-FREE-ROBOT-SEARCH-UPDATE-INTERVAL*))
  (test (eq ?uuid (cx-rl-interfaces-get-free-robot-server-goal-handle-get-goal-id ?ptr)))
=>
  (bind ?feedback (cx-rl-interfaces-get-free-robot-feedback-create))
  (cx-rl-interfaces-get-free-robot-feedback-set-field ?feedback "feedback" "No free robot found, retrying...")
  (cx-rl-interfaces-get-free-robot-server-goal-handle-publish-feedback ?ptr ?feedback)
  (cx-rl-interfaces-get-free-robot-feedback-destroy ?feedback)
  (modify ?gfr (last-search ?now))
)

(defrule get-free-robot-abort
  (cx-rl-node (name ?node))
  ?ag <- (cx-rl-interfaces-get-free-robot-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  (test (str-index ?node ?server))
  ?gfr <- (rl-ros-action-meta-get-free-robot (node ?node) (robot ?robot) (last-search ?last) (found ?found) (uuid ?uuid) (abort-action TRUE))
  (test (eq ?uuid (cx-rl-interfaces-get-free-robot-server-goal-handle-get-goal-id ?ptr)))
=>
  (printout ?*CX-RL-LOG-LEVEL* "ResetCX: Aborting robot search" crlf)
  (bind ?result (cx-rl-interfaces-get-free-robot-result-create))
  (cx-rl-interfaces-get-free-robot-result-set-field ?result "robot" "Aborted")
  (cx-rl-interfaces-get-free-robot-server-goal-handle-abort ?ptr ?result)
  (cx-rl-interfaces-get-free-robot-result-destroy ?result)
  (cx-rl-interfaces-get-free-robot-server-goal-handle-destroy ?ptr)
  (retract ?gfr)
  (retract ?ag)
)

(defrule get-free-robot-search-done
  ?ag <- (cx-rl-interfaces-get-free-robot-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  ?gfr <- (rl-ros-action-meta-get-free-robot (robot ?robot) (last-search ?last) (found TRUE) (uuid ?uuid) (abort-action FALSE))
  (test (eq ?uuid (cx-rl-interfaces-get-free-robot-server-goal-handle-get-goal-id ?ptr)))
=>
  (printout ?*CX-RL-LOG-LEVEL* "Free robot found: " ?robot crlf)
  (bind ?result (cx-rl-interfaces-get-free-robot-result-create))
  (if (neq ?robot "") then
  (cx-rl-interfaces-get-free-robot-result-set-field ?result "robot" ?robot)
  )
  (cx-rl-interfaces-get-free-robot-server-goal-handle-succeed ?ptr ?result)
  (cx-rl-interfaces-get-free-robot-result-destroy ?result)
  (cx-rl-interfaces-get-free-robot-server-goal-handle-destroy ?ptr)
  (retract ?gfr)
  (retract ?ag)
)

(defrule get-free-robot-server-cleanup
  (executive-finalize)
  (cx-rl-interfaces-get-free-robot-server (name ?server))
=>
  (cx-rl-interfaces-get-free-robot-destroy-server ?server)
)

(defrule get-free-robot-accepted-goal-cleanup
  (executive-finalize)
  ?ag <- (cx-rl-interfaces-get-free-robot-accepted-goal (server-goal-handle-ptr ?ptr))
=>
  (cx-rl-interfaces-get-free-robot-server-goal-handle-destroy ?ptr)
  (retract ?ag)
)
