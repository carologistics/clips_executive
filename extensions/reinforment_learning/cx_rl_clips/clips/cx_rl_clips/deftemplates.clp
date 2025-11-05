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
  (multislot objects (type STRING) (default (create$)))
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
