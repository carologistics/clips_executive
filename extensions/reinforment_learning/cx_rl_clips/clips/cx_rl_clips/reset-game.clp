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

(defrule reset-game-stage-zero
  (declare (salience ?*SALIENCE-RESET-GAME-HIGH*))
  ?r <- (reset-game (stage STAGE-0))
  (cx-rl-node (name ?name) (mode ?mode))
  (cx-rl-interfaces-reset-cx-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  =>
  (reset)
  (load-facts reset-save)
  (delayed-do-for-all-facts ((?r rl-action))
    TRUE
    (retract ?r)
  )
  (retract ?r)
  (assert (cx-rl-interfaces-reset-cx-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr)))
  (assert (cx-rl-node (name ?name) (mode ?mode)))
  (assert (reset-game-finished))
  (assert (rl-current-action-space (state PENDING)))
)
