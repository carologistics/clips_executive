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

  ?*SALIENCE-RESET-GAME-HIGH* = 1000
  ?*SALIENCE-RESET-GAME-MIDDLE* = 800
  ?*SALIENCE-RESET-GAME-LOW* = 300
  ?*RESET-GAME-TIMER* = 1.0
)

(deftemplate reset-game
  (slot stage (type SYMBOL))
)

; asseted by user
(deftemplate cx-rl-node
  (slot name (type STRING) (default "/cx_rl_gym"))
  (slot mode (type SYMBOL) (allowed-values TRAINING EXECUTION))
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
  ; note: renamed from points
  (slot reward  (type INTEGER)
                (default 0))
)

; note renamed from rl-action-selection
; not relevant for user
(deftemplate rl-ros-action-meta
	(slot uuid (type STRING))
  (slot action-id (type SYMBOL))
)

; user sets this
; or if user gives action selection without choices
(deftemplate rl-episode-end
  (slot success (type SYMBOL)
                (allowed-values TRUE FALSE)
	              (default TRUE))
)

(deftemplate rl-observable-type
  (slot type (type SYMBOL))
  (multislot objects (type STRING) (default (create$)))
)

(deftemplate rl-observable-predicate
  (slot name (type SYMBOL))
  (multislot param-names (type SYMBOL))
  (multislot param-types (type SYMBOL))
)

; todo: what is predefined osbervable?
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

; System asserts this to PENDING in TRAINING mode
; User asserts the necessary rl-action items and modifies it to DONE
; For Execution:
; User asserts this to DONE to get a suggested action
(deftemplate rl-current-action-space
  (slot state (type SYMBOL) (allowed-values PENDING DONE) (default PENDING))
)

(deftemplate rl-action-request-meta
  (slot service (type STRING))
  (slot request-id (type INTEGER))
  (slot action-id (type SYMBOL))
)

(deffunction delete-rl-actions-after-reset ()
  (delayed-do-for-all-facts ((?r rl-action))
    TRUE
    (retract ?r)
  )
)

(deffunction cx-rl-interfaces-get-free-robot-handle-goal-callback (?server ?goal ?uuid)
    (printout blue ?server " callback (goal " ?goal " ; id " ?uuid " )" crlf)
    (return 2)
)

(deffunction cx-rl-interfaces-get-free-robot-cancel-goal-callback (?server ?goal ?goal-handle)
    (return 1)
)

(deftemplate get-free-robot
    (slot uuid (type STRING))
    (slot robot (type STRING))
    (slot last-search (type FLOAT))
    (slot found (type SYMBOL)
                (allowed-values TRUE FALSE))
)
