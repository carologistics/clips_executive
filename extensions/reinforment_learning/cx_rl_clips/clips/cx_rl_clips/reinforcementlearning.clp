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

(defglobal
  ?*SALIENCE-RL-FIRST* = 10000
  ?*SALIENCE-RL-HIGH* = 1000
  ?*SALIENCE-ROBOT-INIT* = 501
  ?*SALIENCE-ACTION-EXECUTABLE-CHECK* = 500
  ?*SALIENCE-RL-EPISODE-END-SUCCESS* = 500
  ?*SALIENCE-ROBOT-ASSIGNMENT* = 499
  ?*SALIENCE-RL-EPISODE-END-FAILURE* = 499
  ?*SALIENCE-RL-SELECTION* = 498
)

(deftemplate rl-action
  (slot id (type SYMBOL))
  (slot name (type SYMBOL))
  (slot is-selected (type SYMBOL)
                    (allowed-values TRUE FALSE)
                     (default FALSE))
  (slot is-finished (type SYMBOL)
                    (allowed-values TRUE FALSE)
                    (default FALSE))
  (slot assigned-to (type SYMBOL)
                    (default nil))
  (slot points  (type INTEGER)
                (default 0))
)

(deftemplate rl-action-selection
	(slot uuid (type STRING))
  (slot actionid (type SYMBOL))
  (slot is-finished (type SYMBOL)
                    (allowed-values TRUE FALSE)
                    (default FALSE))
  (slot reward  (type INTEGER)
                (default 0))
  (slot done  (type SYMBOL)
              (allowed-values TRUE FALSE)
              (default FALSE))
)

(deftemplate rl-action-selection-exec
  (slot actionid (type SYMBOL))
)

(deftemplate rl-episode-end
  (slot success (type SYMBOL)
                (allowed-values TRUE FALSE)
	              (default TRUE))
)

(deftemplate rl-mode
  (slot mode  (type SYMBOL)
              (allowed-values TRAINING EXECUTION))
)

(deftemplate rl-observable-type
  (slot type (type SYMBOL))
  (multislot objects (type SYMBOL) (default (create$)))
)

(deftemplate rl-observable-predicate
  (slot name (type SYMBOL))
  (multislot param-names (type SYMBOL))
  (multislot param-types (type SYMBOL))
)

(deftemplate rl-predefined-observable
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL))
)

(deftemplate rl-observation
  (slot name (type SYMBOL))
  (multislot param-values (type SYMBOL))
)

(deftemplate rl-robot
  (slot name (type SYMBOL))
  (slot waiting (type SYMBOL) (allowed-values TRUE FALSE) (default TRUE))
)

(deftemplate rl-executability-check
  (slot state (type SYMBOL) (allowed-values PENDING CHECKING CHECKED) (default PENDING))
)

(deftemplate rl-action-selection-requested)

(deffunction rl-action-selected-update-actions ()
  (delayed-do-for-all-facts ((?a rl-action))
		(eq ?a:is-selected FALSE)
		(retract ?a)
	)
)

(deffunction rl-action-selected-update-robots (?robot)
	(if (neq ?robot nil) then
		(delayed-do-for-all-facts ((?a rl-action))
			(eq ?a:mode FORMULATED)
			(modify ?a (assigned-to nil))
		)
	)
)

(defrule rl-action-select
  (declare (salience ?*SALIENCE-RL-SELECTION*))
  ?ec <- (rl-executability-check (state CHECKED))
  (rl-mode (mode TRAINING))
	(rl-action-selection (actionid ?a))
	?next-action <- (rl-action (id ?a) (is-selected FALSE) (is-finished FALSE) (assigned-to ?robot))
  ?rw <- (rl-robot (name ?robot) (waiting TRUE))
	=>
	(printout info crlf "CXRL: Selected action " ?a  "for robot " ?robot crlf )
	(modify ?rw (waiting FALSE))
  (modify ?next-action (is-selected TRUE))
  (rl-action-selected-update-actions)
  (modify ?ec (state PENDING))
  ;(rl-action-selected-update-robots ?robot)

)

(defrule rl-executability-check-before-action-finished
  (declare (salience ?*SALIENCE-ACTION-EXECUTABLE-CHECK*))
  (rl-action (is-finished TRUE))
  ?ec <- (rl-executability-check (state PENDING))
  =>
  (modify ?ec (state CHECKING))
  (rl-action-selected-update-actions)

)
(defrule rl-action-finished
  (declare (salience ?*SALIENCE-RL-SELECTION*))
  (rl-mode (mode TRAINING))
  (rl-executability-check (state CHECKED))
	?r <- (rl-action-selection (actionid ?actionid))
	?a <- (rl-action (id ?actionid) (is-finished TRUE) (points ?points))
	=>
	(printout info crlf "CXRL: Action " ?actionid " has finished" crlf )

  (modify ?r (actionid ?actionid) (is-finished TRUE) (reward ?points) (done FALSE))
  (retract ?a)
)

(defrule rl-action-finished-episode-end
  (declare (salience (+ ?*SALIENCE-RL-SELECTION* 1)))
  (rl-mode (mode TRAINING))
  (rl-executability-check (state CHECKED))
	?r <- (rl-action-selection (actionid ?actionid))
	?a <- (rl-action (id ?actionid) (is-finished TRUE) (points ?points))
  ?e <- (rl-episode-end (success ?success))
	=>
	(printout info crlf "CXRL: Action " ?actionid " has finished, end of episode"crlf )

  (if (eq ?success FALSE) then
    (bind ?reward ?*POINTS-EPISODE-END-FAILURE*)
  else
    (bind ?reward (+ ?points ?*POINTS-EPISODE-END-SUCCESS*))
  )

  (modify ?r (actionid ?actionid) (is-finished TRUE) (reward ?reward) (done TRUE))
  (retract ?e)
  (retract ?a)
)

(defrule domain-game-finished-failure
  (declare (salience ?*SALIENCE-RL-EPISODE-END-FAILURE*))
  (rl-mode (mode TRAINING))
  (rl-executability-check (state CHECKED))
  (rl-action (is-finished TRUE))
  (not (rl-action (is-selected FALSE)))
  (not (rl-episode-end (success ?success)))
  =>
  (assert (rl-episode-end (success FALSE)))
)

(defrule logging-on-episode-end
  (declare (salience ?*SALIENCE-RL-EPISODE-END-SUCCESS*))
  (rl-episode-end (success ?success))
  =>
  (if (eq ?success TRUE) then
    (printout info "END OF EPISODE: SUCCESS" crlf)
  else
    (printout info "END OF EPISODE: FAILURE" crlf)
  )
)

(defrule rl-execution-demand-selection
  (rl-mode (mode EXECUTION))
  (not (rl-action-selection-requested))
  (rl-action (is-selected FALSE) (assigned-to ?robot&~nil))
  (not (rl-episode-end))
  =>
  (assert (rl-action-selection-requested))
)

(defrule rl-action-select-execution
  (declare (salience ?*SALIENCE-RL-SELECTION*))
  (rl-mode (mode EXECUTION))
	?r <- (rl-action-selection-exec (actionid ?actionid))
  ?re <- (rl-action-selection-requested)
	?next-action <- (rl-action (id ?actionid) (is-selected FALSE) (assigned-to ?robot))
  ?rw <- (rl-robot (name ?robot) (waiting TRUE))
	=>
	(printout info crlf "CXRL: Selected action " ?actionid  "for robot " ?robot crlf )

	(retract ?re)
  (retract ?r)
  (modify ?rw (waiting FALSE))
  (modify ?next-action (is-selected TRUE))
  (rl-action-selected-update-actions)
  ;(rl-action-selected-update-robots ?robot)
)

(defrule rl-action-finished-execution
  (declare (salience ?*SALIENCE-RL-SELECTION*))
  (rl-mode (mode EXECUTION))
  ?a <- (rl-action (is-finished TRUE))
  =>
  (retract ?a)
)

;================== ROBOT SELECTION ==================


;(defrule init-robot-waiting
;  (declare (salience ?*SALIENCE-ROBOT-INIT*))
;  (domain-object (name ?robot) (type robot))
;  (not (rl-action (is-selected TRUE) (assigned-to ?robot)))
;  (not (robot-waiting (robot ?robot)))
;  =>
;  (assert (robot-waiting (robot ?robot)))
;)

(defrule assign-robot-to-rl-actions
	" Before checking rl-actions for their executability, pick a waiting robot
  that should get a new action assigned to it next. "
  (declare (salience ?*SALIENCE-ROBOT-ASSIGNMENT*))
  (rl-executability-check (state CHECKED))
  (rl-action (id ?id) (is-selected FALSE) (assigned-to nil))
  (rl-robot (name ?robot) (waiting TRUE))
  (not (rl-action (assigned-to ?robot)))
  (not  (and (rl-robot (name ?robot2&:(neq ?robot2 ?robot)) (waiting TRUE))
            (rl-action (id ?id2) (is-selected FALSE) (assigned-to ?robot2))
        )
  )
  =>
  (bind ?longest-waiting 0)
  (bind ?longest-waiting-robot ?robot)
  (delayed-do-for-all-facts ((?rw rl-robot))
    (eq ?rw:waiting TRUE)
    (if (or (eq ?longest-waiting 0) (< (fact-index ?rw) ?longest-waiting))
     then
      (bind ?longest-waiting-robot ?rw:name)
      (bind ?longest-waiting (fact-index ?rw))
    )
  )
  (delayed-do-for-all-facts ((?a rl-action))
    (and (eq ?a:is-selected FALSE)
         (eq ?a:assigned-to nil))
    (modify ?a (assigned-to ?longest-waiting-robot))
  )
  (retract ?longest-waiting)
  (assert (rl-robot (name ?robot) (waiting TRUE)))
)

(defrule unassign-robot-from-finished-action
  (declare (salience ?*SALIENCE-RL-HIGH*))
  ?a <- (rl-action (is-finished TRUE) (assigned-to ?robot&~nil))
  ?rw <- (rl-robot (name ?robot) (waiting FALSE))
  =>
  (modify ?a (assigned-to nil))
  (modify ?rw (waiting TRUE))
)
