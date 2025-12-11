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

(defrule cx-rl-reset-env-abort-running-actions-abort-action-selection
  (rl-reset-env (node ?node) (state ABORT-RUNNING-ACTIONS))
  (rl-ros-action-meta-action-selection (node ?node) (abort-action FALSE))
  =>
  (delayed-do-for-all-facts ((?meta rl-ros-action-meta-action-selection)) (eq ?meta:node ?node)
    (modify ?meta (abort-action TRUE))
  )
)
(defrule cx-rl-reset-env-abort-running-actions-abort-get-free-robot
  (rl-reset-env (node ?node) (state ABORT-RUNNING-ACTIONS))
  (rl-ros-action-meta-get-free-robot (node ?node) (abort-action FALSE))
  =>
  (delayed-do-for-all-facts ((?meta rl-ros-action-meta-get-free-robot)) (eq ?meta:node ?node)
    (modify ?meta (abort-action TRUE))
  )
)

(defrule cx-rl-reset-env-abort-running-actions-finish
  ?reset-env <- (rl-reset-env (node ?node) (state ABORT-RUNNING-ACTIONS))
  (not (rl-ros-action-meta-get-free-robot (node ?node)))
  (not (rl-ros-action-meta-action-selection (node ?node)))
  =>
  (modify ?reset-env (state USER-CLEANUP))
)


(defrule cx-rl-reset-env-stage-load-facts
  ?r <- (rl-reset-env (node ?node) (uuid ?uuid) (state LOAD-FACTS))
  (cx-rl-node (name ?node) (mode ?mode))
  (cx-rl-interfaces-reset-env-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr))
  (test (str-index ?node ?server))
  =>
  (reset)
  (load-facts reset-save)
  (delayed-do-for-all-facts ((?r rl-action))
    (eq ?r:node ?node)
    (retract ?r)
  )
  (assert (cx-rl-interfaces-reset-env-accepted-goal (server ?server) (server-goal-handle-ptr ?ptr)))
  (assert (cx-rl-node (name ?node) (mode ?mode)))
  (assert (rl-reset-env (node ?node) (uuid ?uuid) (state USER-INIT)))
)
