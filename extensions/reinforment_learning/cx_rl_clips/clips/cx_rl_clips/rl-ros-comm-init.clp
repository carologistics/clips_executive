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

(defrule rl-service-init
" Initiate the service clients for the pddl manager "
  (cx-rl-node (name ?node-name))
  =>
  ; create clints for all services
  (bind ?services ?*CX-RL-SERVICES*)
  (bind ?service-clients ?*CX-RL-SERVICE-CLIENTS*)
  (bind ?index 1)
  (bind ?length (length$ ?services))
  (while (< ?index ?length)
      (bind ?service-name (nth$ ?index ?services))
      (bind ?service-type (nth$ (+ ?index 1) ?services))
      (ros-msgs-create-service
          (str-cat ?node-name "/" ?service-name)
          (str-cat "cx_rl_interfaces/srv/" ?service-type)
      )
      (bind ?index (+ ?index 2))
  )
  (bind ?index 1)
  (bind ?length (length$ ?service-clients))
  (while (< ?index ?length)
      (bind ?service-name (nth$ ?index ?service-clients))
      (bind ?service-type (nth$ (+ ?index 1) ?service-clients))
      (ros-msgs-create-client
          (str-cat ?node-name "/" ?service-name)
          (str-cat "cx_rl_interfaces/srv/" ?service-type)
      )
      (bind ?index (+ ?index 2))
  )
)
