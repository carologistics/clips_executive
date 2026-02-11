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

; Do we need this?
; (defrule init-agent-config
; =>
;   (unwatch facts time)
;   (unwatch rules time-retract)
;   (bind ?share-dir (ament-index-get-package-share-directory "cx_rl_bringup"))
;   (config-load (str-cat ?share-dir "/params/agent_config.yaml") "/")
; )

(defrule init-stuff
=>
  (assert
    (rl-predefined-observable (name on-table) (params block1))
    (rl-predefined-observable (name on-table) (params block2))
    (rl-predefined-observable (name on-table) (params block3))
    (rl-predefined-observable (name on-table) (params block4))
    (rl-observable-predicate (name clear) (param-names a) (param-types block))
    (rl-observable-predicate (name can-hold) (param-names r) (param-types robot))
    (rl-observable-predicate (name holding) (param-names r a) (param-types robot block))
    (rl-observable-predicate (name on) (param-names a b) (param-types block block))
    (rl-observable-predicate (name target-on) (param-names a b) (param-types block block))
    (rl-observable-predicate (name target-on-table) (param-names a b) (param-types block))
    (rl-observable-type (type robot) (objects robot1))
    (rl-observable-type (type block) (objects block1 block2 block3 block4))
    (rl-robot (name robot1) (waiting TRUE))
    (rl-observation (name clear) (param-values block1))
    (rl-observation (name clear) (param-values block2))
    (rl-observation (name clear) (param-values block3))
    (rl-observation (name clear) (param-values block4))
    (rl-observation (name on-table) (param-values block1))
    (rl-observation (name on-table) (param-values block2))
    (rl-observation (name on-table) (param-values block3))
    (rl-observation (name on-table) (param-values block4))
    (rl-observation (name target-on) (param-values block4 block3))
    (rl-observation (name target-on) (param-values block3 block2))
    (rl-observation (name target-on) (param-values block2 block1))
    (rl-observation (name target-on-table) (param-values block1))
    (rl-observation (name can-hold) (param-values robot1))
    (rl-observation (name clear) (param-values block1))
    (rl-observation (name clear) (param-values block2))
    (rl-observation (name clear) (param-values block3))
    (rl-observation (name clear) (param-values block4))
    (rl-observable-action (name stack) (param-names r b1 b2) (param-types robot block block))
    (rl-observable-action (name pickup) (param-names r b) (param-types robot block))
  )
  (if (not (any-factp ((?node cx-rl-node)) (eq ?node:name ?*CX-RL-NODE-NAME*))) then
   (assert (cx-rl-node (name ?*CX-RL-NODE-NAME*) (mode UNSET)))
  )
)

(defrule reset-to-load-facts
 ?reset <- (rl-reset-env (state USER-CLEANUP))
 =>
 (modify ?reset (state LOAD-FACTS))
)

(defrule reset-to-done
 ?reset <- (rl-reset-env (state USER-INIT))
 =>
 (modify ?reset (state DONE))
)

(defrule execution-ask-for-execution
  (cx-rl-node (mode EXECUTION))
  (not (rl-current-action-space))
  =>
  (assert (rl-current-action-space (state PENDING)))
)

(defrule execution-no-action-possible
  (cx-rl-node (mode EXECUTION))
  (rl-robot (name ?robot))
  (rl-action (id no-op) (is-selected TRUE) (assigned-to ?robot))
  =>
  (printout green "Execution done, no more actions to select" crlf)
)

(defrule execution-assign-actions
  (rl-current-action-space (state DONE))
  (cx-rl-node (mode EXECUTION))
  (rl-robot (name ?robot) (waiting TRUE))
  (rl-action (assigned-to nil))
  =>
  (delayed-do-for-all-facts ((?action rl-action))
    (eq ?action:assigned-to nil)
    (modify ?action (assigned-to ?robot))
  )
)

(defrule training-provide-action-stack
  (rl-current-action-space (state PENDING))
  (rl-robot (name ?robot) (waiting TRUE))
  (rl-observation (name holding) (param-values ?robot ?some-block))
  (rl-observation (name clear) (param-values ?other-block))
  (test (neq ?some-block ?other-block))
  =>
  (bind ?block1 (sub-string 6 6 ?some-block))
  (bind ?block2 (sub-string 6 6 ?other-block))
  (bind ?id (sym-cat "stack" ?block1 ?block2))
  (bind ?name (sym-cat "stack(" ?robot "#" ?some-block "#" ?other-block ")"))
  (assert (rl-action (id ?id) (name ?name)))
)

(defrule training-provide-action-pickup
  (rl-current-action-space (state PENDING))
  (rl-robot (name ?robot) (waiting TRUE))
  (rl-observation (name can-hold) (param-values ?robot))
  (rl-observation (name on-table) (param-values ?some-block))
  (rl-observation (name clear) (param-values ?some-block))
  =>
  (bind ?block1 (sub-string 6 6 ?some-block))
  (bind ?id (sym-cat "pickup" (gensym*)))
  (bind ?name (sym-cat "pickup(" ?robot "#" ?some-block ")"))
  (assert (rl-action (id ?id) (name ?name)))
)

(defrule training-actions-generation-done
  (declare (salience -1000))
  ?action-space <- (rl-current-action-space (state PENDING))
  =>
  (modify ?action-space (state DONE))
)

(defrule action-selected-action-done-pickup
  (rl-robot (name ?robot))
  ?obs1 <- (rl-observation (name can-hold) (param-values ?robot))
  ?obs2 <- (rl-observation (name on-table) (param-values ?block))
  ?action <- (rl-action (name ?name&:(str-index ?block ?name)) (is-selected TRUE) (is-finished FALSE))
  =>
  (retract ?obs1 ?obs2)
  (assert (rl-observation (name holding) (param-values ?robot ?block)))
  (bind ?reward 0)
  (do-for-fact ((?target rl-observation))
    (and (eq ?target:name target-on-table)
         (eq ?target:param-values (create$ ?block)))
    (printout red "useless action pickup "?target:param-values crlf)
    (bind ?reward -100)
  )
  (modify ?action (is-finished TRUE) (reward ?reward))
)

(defrule action-selected-action-done-stack
  (rl-robot (name ?robot))
  ?obs1 <- (rl-observation (name holding) (param-values ?robot ?block))
  (rl-observable-type (type block) (objects $? ?other-block&:(neq ?other-block ?block) $?))
  ?obs2 <- (rl-observation (name clear) (param-values ?other-block2&:(eq ?other-block2 (sym-cat ?other-block))))
  ?action <- (rl-action (name ?name) (is-selected TRUE) (is-finished FALSE))
  (test (and (str-index stack ?name) (str-index ?robot ?name) (str-index ?block ?name) (str-index ?other-block ?name)))
  =>
  (retract ?obs1 ?obs2)
  (assert (rl-observation (name on) (param-values ?block (sym-cat ?other-block))))
  (assert (rl-observation (name can-hold) (param-values ?robot)))
  (bind ?reward -100)
  (do-for-fact ((?target rl-observation))
    (and (eq ?target:name target-on)
         (eq ?target:param-values (create$ ?block (sym-cat ?other-block))))
    (printout green "useful action on "?target:param-values crlf)
    (bind ?reward 100)
  )
  (modify ?action (is-finished TRUE) (reward ?reward))
)

(defrule blocksworld-episode-end-succes
  (declare (salience 10000))
  (rl-observation (name target-on) (param-values $?target1))
  (rl-observation (name target-on) (param-values $?target2))
  (rl-observation (name target-on) (param-values $?target3))
  (test (and (neq ?target1 ?target2) (neq ?target1 ?target3) (neq ?target2 ?target3)))
  (rl-observation (name on) (param-values $?target1))
  (rl-observation (name on) (param-values $?target2))
  (rl-observation (name on) (param-values $?target3))
  ?action <- (rl-action (name ?name) (is-selected TRUE) (is-finished TRUE))
  (not (rl-episode-end (success TRUE)))
  =>
  (printout green "SUCCESS!!!!!!" crlf)
  (assert (rl-episode-end (success TRUE)))
  (modify ?action (reward 1000))
)

(defrule rl-stop-agent-on-training-end
  (rl-end-training)
  =>
  (cx-shutdown)
)
