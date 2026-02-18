; Copyright (c) 2026 Carologistics
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

; Step 0: Configuration via Global Variables

(defglobal
  ?*CX-RL-REWARD-EPISODE-SUCCESS* = 100
  ?*CX-RL-REWARD-EPISODE-FAILURE* = -100
)

; Step 1: Defining the Environment

(defrule rl-blocksworld-initial-state
  (not (cx-rl-node))
=>
  (assert
    (rl-observable-type (type robot) (objects robot1))
    (rl-observable-type (type block) (objects block1 block2 block3 block4))
    (rl-observable-predicate (name on-table) (param-names a) (param-types block))
    (rl-observable-predicate (name clear) (param-names a) (param-types block))
    (rl-observable-predicate (name can-hold) (param-names r) (param-types robot))
    (rl-observable-predicate (name holding) (param-names r a) (param-types robot block))
    (rl-observable-predicate (name on) (param-names a b) (param-types block block))
    (rl-observable-predicate (name target-on) (param-names a b) (param-types block block))
    (rl-observable-predicate (name target-on-table) (param-names a) (param-types block))
    (rl-observation (name clear) (params block1))
    (rl-observation (name clear) (params block2))
    (rl-observation (name clear) (params block3))
    (rl-observation (name clear) (params block4))
    (rl-observation (name on-table) (params block1))
    (rl-observation (name on-table) (params block2))
    (rl-observation (name on-table) (params block3))
    (rl-observation (name on-table) (params block4))
    (rl-observation (name target-on) (params block4 block3))
    (rl-observation (name target-on) (params block3 block2))
    (rl-observation (name target-on) (params block2 block1))
    (rl-observation (name target-on-table) (params block1))
    (rl-observation (name can-hold) (params robot1))
    (rl-observation (name clear) (params block1))
    (rl-observation (name clear) (params block2))
    (rl-observation (name clear) (params block3))
    (rl-observation (name clear) (params block4))
    (rl-observable-action (name stack) (param-names r b1 b2) (param-types robot block block))
    (rl-observable-action (name pickup) (param-names r b) (param-types robot block))
    (rl-predefined-action (name pickup) (params robot1 block1))
    (rl-predefined-action (name pickup) (params robot1 block2))
    (rl-predefined-action (name pickup) (params robot1 block3))
    (rl-predefined-action (name pickup) (params robot1 block4))
    (rl-robot (name robot1) (waiting TRUE))
  )
  (assert (cx-rl-node (name ?*CX-RL-NODE-NAME*) (mode UNSET)))
)

; Step 2: Defining the Reset Procedure

(defrule rl-blocksworld-reset-to-load-facts
 ?reset <- (rl-reset-env (state USER-CLEANUP))
 =>
 (modify ?reset (state LOAD-FACTS))
)

(defrule rl-blocksworld-reset-to-done
 ?reset <- (rl-reset-env (state USER-INIT))
 =>
 (modify ?reset (state DONE))
)

; Step 3: Action Execution

; providing actions given a pending current action space

(defrule rl-blocksworld-provide-action-stack
  (rl-current-action-space (state PENDING))
  (rl-robot (name ?robot) (waiting TRUE))
  (rl-observation (name holding) (params ?robot ?some-block))
  (rl-observation (name clear) (params ?other-block))
  (test (neq ?some-block ?other-block))
=>
  (bind ?id (sym-cat "stack" (gensym*)))
  (assert (rl-action (id ?id) (name stack) (params ?robot ?some-block ?other-block)))
)

(defrule rl-blocksworld-provide-action-pickup
  (rl-current-action-space (state PENDING))
  (rl-robot (name ?robot) (waiting TRUE))
  (rl-observation (name can-hold) (params ?robot))
  (rl-observation (name on-table) (params ?some-block))
  (rl-observation (name clear) (params ?some-block))
=>
  (bind ?id (sym-cat "pickup" (gensym*)))
  (assert (rl-action (id ?id) (name pickup) (params ?robot ?some-block)))
)

(defrule rl-blocksworld-actions-generation-done
  (declare (salience -2))
  ?action-space <- (rl-current-action-space (state PENDING))
=>
  (modify ?action-space (state DONE))
)

; handling selected actions, applying rewards and updated observations

(defrule action-selected-action-done-stack
  (rl-robot (name ?robot))
  ?obs1 <- (rl-observation (name holding) (params ?robot ?block))
  ?action <- (rl-action (name stack) (params ?robot ?block ?other-block) (is-selected TRUE) (is-finished FALSE))
  (rl-observable-type (type block) (objects $? ?other-block&:(neq ?other-block ?block) $?))
  ?obs2 <- (rl-observation (name clear) (params ?other-block2&:(eq ?other-block2 (sym-cat ?other-block))))
=>
  (retract ?obs1 ?obs2)
  (assert (rl-observation (name on) (params ?block (sym-cat ?other-block))))
  (assert (rl-observation (name can-hold) (params ?robot)))
  (bind ?reward 0)
  (do-for-fact ((?target rl-observation))
    (and (eq ?target:name target-on)
         (eq ?target:params (create$ ?block (sym-cat ?other-block))))
    (printout green "useful action on "?target:params crlf)
    (bind ?reward 100)
  )
  (modify ?action (is-finished TRUE) (reward ?reward))
)

(defrule action-selected-action-done-pickup
  (rl-robot (name ?robot))
  ?obs1 <- (rl-observation (name can-hold) (params ?robot))
  ?obs2 <- (rl-observation (name on-table) (params ?block))
  ?action <- (rl-action (name pickup) (params ?robot ?block) (is-selected TRUE) (is-finished FALSE))
=>
  (retract ?obs1 ?obs2)
  (assert (rl-observation (name holding) (params ?robot ?block)))
  (bind ?reward 0)
  (do-for-fact ((?target rl-observation))
    (and (eq ?target:name target-on-table)
         (eq ?target:params (create$ ?block)))
    (printout red "useless action pickup "?target:params crlf)
    (bind ?reward -100)
  )
  (modify ?action (is-finished TRUE) (reward ?reward))
)

; Training

(defrule rl-blocksworld-episode-end-success
  (declare (salience 1))
  (rl-observation (name target-on) (params $?target1))
  (rl-observation (name target-on) (params $?target2))
  (rl-observation (name target-on) (params $?target3))
  (test (and (neq ?target1 ?target2) (neq ?target1 ?target3) (neq ?target2 ?target3)))
  (rl-observation (name on) (params $?target1))
  (rl-observation (name on) (params $?target2))
  (rl-observation (name on) (params $?target3))
  ?action <- (rl-action (name ?name) (is-selected TRUE) (is-finished TRUE))
  (not (rl-episode-end (success TRUE)))
=>
  (printout green "SUCCESS!" crlf)
  (assert (rl-episode-end (success TRUE)))
  (modify ?action (reward 1000))
)

(defrule rl-blocksworld-episode-end-failure
  (declare (salience -1))
  (cx-rl-node (name ?node) (mode TRAINING))
  (rl-current-action-space (node ?node) (state PENDING))
  (not (rl-action (node ?node) (is-selected FALSE)))
  (not (rl-episode-end (node ?node) (success ?success)))
=>
  (assert (rl-episode-end (node ?node) (success FALSE)))
)

(defrule rl-blocksworld-stop-agent-on-training-end
  (rl-end-training)
=>
  (cx-shutdown)
)

; Execution

(defrule rl-blocksworld-ask-for-execution
  (cx-rl-node (mode EXECUTION))
  (not (rl-current-action-space))
  (not (rl-action (is-selected TRUE) (is-finished FALSE)))
=>
  (assert (rl-current-action-space (state PENDING)))
)
