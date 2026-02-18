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
"(deffunction " ?*CX-RL-NODE-NAME* "/get_current_observations-service-callback (?service-name ?request ?response)
  (bind ?obs-list (create$))
  (do-for-all-facts ((?obs rl-observation)) (eq ?obs:node ?*CX-RL-NODE-NAME*)
      (bind ?observation-msg (ros-msgs-create-message \"cx_rl_interfaces/msg/Observation\"))
      (ros-msgs-set-field ?observation-msg \"name\" ?obs:name)
      (ros-msgs-set-field ?observation-msg \"params\" ?obs:params)
      (bind ?obs-list (insert$ ?obs-list 1 ?observation-msg))
  )
  (ros-msgs-set-field ?response \"observations\" ?obs-list)
  (foreach ?observation-msg-item ?obs-list
    (ros-msgs-destroy-message ?observation-msg-item)
  )
)"
)
)
