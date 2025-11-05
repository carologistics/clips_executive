

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

; override pddl-action to our needs
(deftemplate pddl-action
  (slot instance (type SYMBOL))
  (slot id (type SYMBOL)) ; this should be a globally unique ID
  (slot plan (type SYMBOL))
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL) (default (create$)))
  (slot plan-order-class (type INTEGER))
  (slot planned-start-time (type FLOAT))
  (slot planned-duration (type FLOAT))
  (slot actual-start-time (type FLOAT))
  (slot actual-duration (type FLOAT))
  (slot state (type SYMBOL) (allowed-values IDLE SELECTED EXECUTING DONE))
)
