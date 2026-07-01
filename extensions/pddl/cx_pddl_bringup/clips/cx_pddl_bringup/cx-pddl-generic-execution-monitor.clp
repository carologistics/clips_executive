; Copyright (c) 2025-2026 Carologistics
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

(defrule cx-pddl-clips-agent-replan-on-unsat
  ?plan <- (pddl-plan (id ?plan-id) (plan-start ?t))
  (pddl-action-condition (action ?action-id) (state CONDITION-UNSAT))
  (pddl-action (id ?action-id) (plan ?plan-id))
  (not (pddl-action-executor (state ACCEPTED)))
  =>
  (modify ?plan (state PENDING))
  (do-for-all-facts ((?pa pddl-action)) (eq ?pa:plan ?plan-id)
       (retract ?pa)
  )
)
