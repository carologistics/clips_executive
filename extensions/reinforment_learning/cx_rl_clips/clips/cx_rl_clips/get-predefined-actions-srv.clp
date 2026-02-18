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
(str-cat
"(deffunction " ?*CX-RL-NODE-NAME* "/get_predefined_actions-service-callback (?service-name ?request ?response)
    (printout ?*CX-RL-LOG-LEVEL* \"Collecting cx rl predefined actions\" crlf)
    (bind ?action-list (create$))

    (do-for-all-facts ((?pa rl-predefined-action))
            (eq ?pa:node \"" ?*CX-RL-NODE-NAME* "\")
      (bind ?action-msg (ros-msgs-create-message \"cx_rl_interfaces/msg/Action\"))
      (ros-msgs-set-field ?action-msg \"name\" ?pa:name)
      (ros-msgs-set-field ?action-msg \"params\" ?pa:params)
      (bind ?action-list (insert$ ?action-list 1 ?action-msg))
    )
    (ros-msgs-set-field ?response \"actions\" ?action-list)
    (foreach ?action-msg-item ?action-list
      (ros-msgs-destroy-message ?action-msg-item)
    )
)"
))
