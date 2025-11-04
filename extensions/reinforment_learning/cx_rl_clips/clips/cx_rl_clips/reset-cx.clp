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
  ?*SALIENCE-RESET-CX-HIGH* = 1000
  ?*SALIENCE-RESET-CX-MIDDLE* = 800
  ?*SALIENCE-RESET-CX-LOW* = 300
  ?*RESET-CX-TIMER* = 1.0
)

(deftemplate reset-cx
 	(slot stage (type SYMBOL))
)

(deftemplate reset-cx-action
    (slot uuid (type STRING))
)

(deffunction delete-rl-actions-after-reset ()
  (delayed-do-for-all-facts ((?r rl-action))
    TRUE
    (retract ?r)
  )
)

(defrule reset-cx-stage-init
  (declare (salience ?*SALIENCE-RESET-CX-HIGH*))
  ?r <- (reset-cx (stage INIT))
  (rl-mode (mode ?mode))
  (cx-rl-interfaces-reset-cx-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  =>
  (modify ?r (stage PRE-RESET))
  
)

(defrule reset-cx-stage-reset
  ?r <- (reset-cx (stage RESET))
  (rl-mode (mode ?mode))
  (cx-rl-interfaces-reset-cx-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  (reset-cx-action (uuid ?uuid))
  =>
  (reset)
  (load-facts reset-save)
  (delete-rl-actions-after-reset)
  (assert (reset-cx (stage POST-RESET)))
  (assert (cx-rl-interfaces-reset-cx-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr)))
  (assert (reset-cx-action (uuid ?uuid)))
  (assert (rl-mode (mode ?mode)))
)

(defrule reset-cx-stage-finalize
  ?r <- (reset-cx (stage FINALIZE))
  =>
  (assert (reset-cx-finished))
  (assert (rl-executability-check (state CHECKING)))
  (retract ?r)
)