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

(defrule unassign-robot-from-finished-action
  (declare (salience ?*SALIENCE-RL-HIGH*))
  ?a <- (rl-action (is-finished TRUE) (assigned-to ?robot&~nil))
  ?rw <- (rl-robot (name ?robot) (waiting FALSE))
  =>
  (modify ?a (assigned-to nil))
  (modify ?rw (waiting TRUE))
)

;================== ROBOT SELECTION ==================

; commented out by frank
;(defrule init-robot-waiting
;  (declare (salience ?*SALIENCE-ROBOT-INIT*))
;  (domain-object (name ?robot) (type robot))
;  (not (rl-action (is-selected TRUE) (assigned-to ?robot)))
;  (not (robot-waiting (robot ?robot)))
;  =>
;  (assert (robot-waiting (robot ?robot)))
;)
