
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

(deftemplate pddl-service-request-meta
  (slot service (type STRING))
  (slot request-id (type INTEGER))
  (slot meta (type SYMBOL))
)

(deftemplate pddl-manager
" Store information on the external pddl manager.
"
  (slot node (type STRING) (default "/pddl_manager"))
)

(deftemplate pddl-action
" Represents a grounded pddl action in a pddl instance.
  @slot instance: pddl instance belonging to the action.
  @slot name: name of the action.
  @slot params: parameters of the  action.
  @slot plan-order-class: partial order class of the action in theplan
  @slot: planned-start-time: start time of the action in the plan
  @slot: planned-duration: assumed duration of the action according to plan
"
  (slot instance (type SYMBOL))
  (slot id (type SYMBOL)) ; this should be a globally unique ID
  (slot plan (type SYMBOL))
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL) (default (create$)))
  (slot plan-order-class (type INTEGER))
  (slot planned-start-time (type FLOAT))
  (slot planned-duration (type FLOAT))
)

(deftemplate pddl-goal-fluent
" Facts to represent goal conditions for planning.
  Each fact of this template represent one positive boolean fluent in a
  goal condition.
  Negative goal conditions are currently not supported.
"
  (slot instance (type SYMBOL))
  (slot goal (type SYMBOL))
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL) (default (create$)))
)

(deftemplate pddl-goal-numeric-fluent
" Facts to represent goal conditions for planning.
  Each fact of this template represent one numeric fluent in a goal condition
  with exactly the value specified in here.
  Note that this is a rather limited representation for numeric fluent
  conditions and can therefore only represent a subset of valid conditions.
"
  (slot instance (type SYMBOL))
  (slot goal (type SYMBOL))
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL) (default (create$)))
  (slot value (type FLOAT))
)

(deftemplate pddl-effect-fluent
" Facts to represent pddl action effects.
  Each fact of this template represent one boolean fluent in an action effect.
"
  (slot instance (type SYMBOL))
  (slot action (type SYMBOL))
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL) (default (create$)))
  (slot effect-type (type SYMBOL) (allowed-values ALL START END) (default ALL))
)

(deftemplate pddl-effect-numeric-fluent
" Facts to represent pddl action effects.
  Each fact of this template represent one numeric fluent in an action effect at a specific time.
  Note that this is a rather limited representation for numeric fluent
  conditions and can therefore only represent a subset of valid conditions.
"
  (slot instance (type SYMBOL))
  (slot action (type SYMBOL))
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL) (default (create$)))
  (slot value (type FLOAT))
  (slot effect-type (type SYMBOL) (allowed-values ALL START END) (default ALL))
)



(deftemplate pddl-fluent
" Represents a (boolean) fluent in a pddl instance.
  Do not retract/assert/modify facts of this type directly, rather use
  request-pddl-fluent facts to indicate the desired change.
  This ensures that the external pddl manager stays in sync with the CLIPS
  model.
  @slot instance: pddl instance to add the fluent to.
  @slot name: name of the fluent.
  @slot params: parameters of the fluent.
"
  (slot instance (type SYMBOL))
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL) (default (create$)))
)

(deftemplate pddl-numeric-fluent
" Represents a numeric fluent in a pddl instance.
  Do not retract/assert/modify facts of this type directly, rather use
  request-pddl-numeric-fluent facts to indicate the desired change.
  This ensures that the external pddl manager stays in sync with the CLIPS
  model.
  @slot instance: pddl instance to add the fluent to.
  @slot name: name of the fluent.
  @slot params: parameters of the fluent.
  @slot value: value of the fluent.
"
  (slot instance (type SYMBOL))
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL) (default (create$)))
  (slot value (type FLOAT))
)

(deftemplate pddl-predicate
" Represents a predicate in a pddl instance.
  @slot instance: pddl instance to the predicate is part of.
  @slot name: name of the predicate.
  @slot param-types: parameter types of the predicate.
"
  (slot instance (type SYMBOL))
  (slot name (type SYMBOL))
  (multislot param-types (type SYMBOL) (default (create$)))
  (multislot param-names (type SYMBOL) (default (create$)))
)

(deftemplate pddl-type-objects
" Lists all objects of a certain type in a pddl instance.
  @slot instance: pddl instance to the object is part of.
  @slot type: type of the object.
  @slot objects: list of the object names.
"
  (slot instance (type SYMBOL))
  (slot type (type SYMBOL))
  (multislot objects (type STRING) (default (create$)))
)

(deftemplate pddl-plan
  (slot id (type SYMBOL))
  (slot instance (type SYMBOL))
  (slot duration (type FLOAT))
  (slot state (type SYMBOL) (allowed-values PENDING SELECTED EXECUTING) (default PENDING))
  (slot context (type SYMBOL) (default nil))
)
