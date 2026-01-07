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

(defglobal
  ?*CX-RL-LOG-LEVEL* = info

  ?*CX-RL-GET-FREE-ROBOT-SEARCH-UPDATE-INTERVAL* = 5
  ?*RESET-GAME-TIMER* = 1.0

  ?*REWARD-EPISODE-SUCCESS* = 100
  ?*REWARD-EPISODE-FAILURE* = -100

  ?*CX-RL-SERVICES* = (create$
      create_rl_action_space GetActionList
      create_rl_env_state CreateRLEnvState
      get_action_list_executable_for_robot GetActionListRobot
      get_action_list_executable GetActionList
      get_episode_end GetEpisodeEnd
      get_observable_objects GetObservableObjects
      get_observable_predicates GetObservablePredicates
      get_predefined_observables GetPredefinedObservables
      set_rl_mode SetRLMode
  )
  ?*CX-RL-SERVICE-CLIENTS* = (create$
      exec_action_selection ExecActionSelection
  )
  ?*CX-RL-ACTION-SERVERS* = (create$
      cx-rl-interfaces-get-free-robot-server get_free_robot
      cx-rl-interfaces-action-selection-server action_selection
      cx-rl-interfaces-reset-env-server reset_env
  )
)

(deftemplate rl-reset-env
" Asserted by the environment."
  (slot state (type SYMBOL)
    (allowed-values ABORT-RUNNING-ACTIONS USER-CLEANUP LOAD-FACTS USER-INIT DONE))
  (slot uuid (type STRING))
  (slot node (type STRING) (default "/cx_rl_node"))
)

(deftemplate cx-rl-node
" Assert this once everything is initialized.
  This will cause the current fact base to be saved as a starting point for
  each episode in training.
"
  (slot name (type STRING) (default "/cx_rl_node"))
  (slot mode (type SYMBOL) (allowed-values TRAINING EXECUTION))
)

(deftemplate rl-action-space
" Define all action names. "
  (slot node (type STRING) (default "/cx_rl_node"))
  (multislot action-names (type STRING))
)

(deftemplate rl-episode-end
" Assert this during training if the episode is supposed to end. "
  (slot node (type STRING) (default "/cx_rl_node"))
  (slot success (type SYMBOL)
    (allowed-values TRUE FALSE)
    (default TRUE))
)

(deftemplate rl-observable-type
" Used to span a symbolic observation space.
  Define a type and the belonging objects with a fact of this kind.
  See the rl-observable-predicate template of how to then define predicates.
  Example: a type BLOCK with objects A B C D.
"
  (slot node (type STRING) (default "/cx_rl_node"))
  (slot type (type SYMBOL))
  (multislot objects (type STRING) (default (create$)))
)

(deftemplate rl-observable-predicate
" Used to span a symbolic observation space.
  Define a predicate with parameters and belonging types.
  The observation space will contain all possible groundings of this predicate
  accoridng to the belonging objects as defined via rl-observable-type.
  Example: name on, param-names (blocka block b) param-types (BLOCK BLOCK).
"
  (slot node (type STRING) (default "/cx_rl_node"))
  (slot name (type SYMBOL))
  (multislot param-names (type SYMBOL))
  (multislot param-types (type SYMBOL))
)

(deftemplate rl-predefined-observable
" Used to span a symbolic observation space.
  Use this to directly assert grounded observables.
"
  (slot node (type STRING) (default "/cx_rl_node"))
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL))
)

(deftemplate rl-observation
" Represent a current observation.
  The set of rl-observation facts hence forms a subset of the observables
  defined in the observation space.
"
  (slot node (type STRING) (default "/cx_rl_node"))
  (slot name (type SYMBOL))
  (multislot param-values (type SYMBOL))
)

(deftemplate rl-robot
" Represents a robot that can perform actions.
  Th waiting slot is handled automatically.
"
  (slot node (type STRING) (default "/cx_rl_node"))
  (slot name (type SYMBOL))
  (slot waiting (type SYMBOL) (allowed-values TRUE FALSE) (default TRUE))
)

(deftemplate rl-current-action-space
" Asserted automatically with state PENDING.
  Populate the current action space by asserting rl-action facts and then
  set the state to DONE.
  During training, if no rl-action facts are present when the state is DONE,
  the episode ends with a failure.
"
  (slot node (type STRING) (default "/cx_rl_node"))
  (slot state (type SYMBOL) (allowed-values PENDING DONE) (default PENDING))
)


(deftemplate rl-action
" Defines an action that can be executed.
  The name needs to match one defined via the rl-action-space fact.
  The ID should be unique to distinguish actions with the same signature.
  The slots is-selected and assigned-to are handled automatically.
  Once an action is selected, the user has to take care of the execution.
  Indicate the outcome by setting is-finished to TRUE and adding a reward,
  if an agent is trained.
"
  (slot id (type SYMBOL))
  (slot node (type STRING) (default "/cx_rl_node"))
  (slot name (type SYMBOL))
  (slot is-selected (type SYMBOL)
                    (allowed-values TRUE FALSE)
                     (default FALSE))
  (slot is-finished (type SYMBOL)
                    (allowed-values TRUE FALSE)
                    (default FALSE))
  (slot assigned-to (type SYMBOL)
                    (default nil))
  (slot reward  (type INTEGER)
                (default 0))
)

(deftemplate rl-ros-action-meta-get-free-robot
  (slot uuid (type STRING))
  (slot node (type STRING) (default "/cx_rl_node"))
  (slot robot (type STRING))
  (slot last-search (type FLOAT))
  (slot found (type SYMBOL)
    (allowed-values TRUE FALSE))
  (slot abort-action (type SYMBOL) (allowed-values FALSE TRUE) (default FALSE))
)

(deftemplate rl-ros-action-meta-action-selection
  (slot uuid (type STRING))
  (slot node (type STRING) (default "/cx_rl_node"))
  (slot action-id (type SYMBOL))
  (slot abort-action (type SYMBOL) (allowed-values FALSE TRUE) (default FALSE))
)

(deftemplate rl-action-request-meta
  (slot node (type STRING) (default "/cx_rl_node"))
  (slot service (type STRING))
  (slot request-id (type INTEGER))
  (slot action-id (type SYMBOL))
)
