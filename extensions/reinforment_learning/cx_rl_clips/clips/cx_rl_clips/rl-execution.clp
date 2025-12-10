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

(defrule cx-rl-assign-robot-to-rl-actions
" Before checking rl-actions for their executability, pick a waiting robot
  that should get a new action assigned to it next. "
  (rl-current-action-space (node ?node) (state DONE))
  (rl-action (node ?node) (id ?id) (is-selected FALSE) (assigned-to nil))
  (rl-robot (node ?node) (name ?robot) (waiting TRUE))
  (not (rl-action (node ?node) (assigned-to ?robot)))
  (not  (and (rl-robot (node ?node) (name ?robot2&:(neq ?robot2 ?robot)) (waiting TRUE))
            (rl-action (node ?node) (id ?id2) (is-selected FALSE) (assigned-to ?robot2))
        )
  )
  =>
  (bind ?longest-waiting 0)
  (bind ?longest-waiting-robot ?robot)
  (delayed-do-for-all-facts ((?rw rl-robot))
    (and (eq ?rw:waiting TRUE) (eq ?rw:node ?node))
    (if (or (eq ?longest-waiting 0) (< (fact-index ?rw) ?longest-waiting))
     then
      (bind ?longest-waiting-robot ?rw:name)
      (bind ?longest-waiting (fact-index ?rw))
    )
  )
  (delayed-do-for-all-facts ((?a rl-action))
    (and (eq ?a:is-selected FALSE)
         (eq ?a:node ?node)
         (eq ?a:assigned-to nil))
    (modify ?a (assigned-to ?longest-waiting-robot))
  )
  (retract ?longest-waiting)
  (assert (rl-robot (node ?node) (name ?robot) (waiting TRUE)))
)

(defrule cx-rl-action-select-training
  ?ec <- (rl-current-action-space (node ?node) (state DONE))
  (cx-rl-node (name ?node) (mode TRAINING))
  (rl-ros-action-meta-action-selection (node ?node) (action-id ?a-id))
  ?next-action <- (rl-action (node ?node) (id ?a-id) (is-selected FALSE) (is-finished FALSE) (assigned-to ?robot))
  ?rw <- (rl-robot (node ?node) (name ?robot) (waiting TRUE))
=>
  (printout ?*CX-RL-LOG-LEVEL* "CXRL: Selected action " ?a-id  "for robot " ?robot crlf)
  (modify ?rw (waiting FALSE))
  (modify ?next-action (is-selected TRUE))
  (delayed-do-for-all-facts ((?a rl-action))
    (and (eq ?a:node ?node) (eq ?a:is-selected FALSE))
    (retract ?a)
  )
  (modify ?ec (state PENDING))
)

(defrule cx-rl-unassign-robot-from-finished-action
  ?a <- (rl-action (node ?node) (id ?action-id) (is-finished TRUE) (assigned-to ?robot&~nil))
  ?rw <- (rl-robot (node ?node) (name ?robot) (waiting FALSE))
  (not (rl-ros-action-meta-action-selection (node ?node) (action-id ?action-id)))
  =>
  (modify ?a (assigned-to nil))
  (modify ?rw (waiting TRUE))
)

(defrule cx-rl-episode-end-failure
  (cx-rl-node (name ?node) (mode TRAINING))
  (rl-current-action-space (node ?node) (state DONE))
  (not (rl-action (node ?node) (is-selected FALSE)))
  (not (rl-episode-end (node ?node) (success ?success)))
  =>
  (assert (rl-episode-end (node ?node) (success FALSE)))
)
