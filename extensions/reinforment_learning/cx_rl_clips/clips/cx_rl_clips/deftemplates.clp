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
  ?*CX-RL-LOG-LEVEL* = info

  ?*CX-RL-GET-FREE-ROBOT-SEARCH-UPDATE-INTERVAL* = 5
  ?*RESET-GAME-TIMER* = 1.0

  ?*REWARD-EPISODE-SUCCESS* = 100
  ?*REWARD-EPISODE-FAILURE* = -100

  ?*CX-RL-SERVICES* = (create$
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
  (slot state (type SYMBOL)
    (allowed-values ABORT-RUNNING-ACTIONS USER-CLEANUP LOAD-FACTS USER-INIT DONE))
  (slot uuid (type STRING))
  (slot node (type STRING) (default "/cx_rl_node"))
)

(deftemplate cx-rl-node
  (slot name (type STRING) (default "/cx_rl_node"))
  (slot mode (type SYMBOL) (allowed-values TRAINING EXECUTION))
)

(deftemplate rl-action
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

(deftemplate rl-episode-end
  (slot node (type STRING) (default "/cx_rl_node"))
  (slot success (type SYMBOL)
    (allowed-values TRUE FALSE)
    (default TRUE))
)

(deftemplate rl-observable-type
  (slot node (type STRING) (default "/cx_rl_node"))
  (slot type (type SYMBOL))
  (multislot objects (type STRING) (default (create$)))
)

(deftemplate rl-observable-predicate
  (slot node (type STRING) (default "/cx_rl_node"))
  (slot name (type SYMBOL))
  (multislot param-names (type SYMBOL))
  (multislot param-types (type SYMBOL))
)

(deftemplate rl-predefined-observable
  (slot node (type STRING) (default "/cx_rl_node"))
  (slot name (type SYMBOL))
  (multislot params (type SYMBOL))
)

(deftemplate rl-observation
  (slot node (type STRING) (default "/cx_rl_node"))
  (slot name (type SYMBOL))
  (multislot param-values (type SYMBOL))
)

(deftemplate rl-robot
  (slot node (type STRING) (default "/cx_rl_node"))
  (slot name (type SYMBOL))
  (slot waiting (type SYMBOL) (allowed-values TRUE FALSE) (default TRUE))
)

(deftemplate rl-current-action-space
  (slot node (type STRING) (default "/cx_rl_node"))
  (slot state (type SYMBOL) (allowed-values PENDING DONE) (default PENDING))
)

(deftemplate rl-action-request-meta
  (slot node (type STRING) (default "/cx_rl_node"))
  (slot service (type STRING))
  (slot request-id (type INTEGER))
  (slot action-id (type SYMBOL))
)
