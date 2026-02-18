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

(build
(str-cat "(deffunction " ?*CX-RL-NODE-NAME* "/get_action_list_executable_for_robot-service-callback (?service-name ?request ?response)
  (bind ?robot (ros-msgs-get-field ?request \"robot\"))
  (printout ?*CX-RL-LOG-LEVEL* \"Generating list of all executable actions for \" ?robot crlf)
  (bind ?temp-action-list (create$))
  (bind ?action-id-list (create$))
  (bind ?action-list (create$))
  (do-for-all-facts ((?action rl-action))
          (and  (eq ?action:is-selected FALSE)
                (eq ?action:node \"" ?*CX-RL-NODE-NAME* "\")
                (eq ?action:assigned-to (sym-cat ?robot)))
      (bind ?action-msg (ros-msgs-create-message \"cx_rl_interfaces/msg/Action\"))
      (ros-msgs-set-field ?action-msg \"name\" ?action:name)
      (ros-msgs-set-field ?action-msg \"params\" ?action:params)
      (bind ?action-list (insert$ ?action-list 1 ?action-msg))
      (bind ?action-id-list (insert$ ?action-id-list 1 ?action:id))
      (bind ?temp-action-list (insert$ ?temp-action-list 1 ?action-msg))
  )
  (printout ?*CX-RL-LOG-LEVEL* ?action-list crlf)
  (ros-msgs-set-field ?response \"actions\" ?action-list)
  (ros-msgs-set-field ?response \"action_ids\" ?action-id-list)
  (foreach ?action-msg-item ?temp-action-list
    (ros-msgs-destroy-message ?action-msg-item)
  )
)"))
