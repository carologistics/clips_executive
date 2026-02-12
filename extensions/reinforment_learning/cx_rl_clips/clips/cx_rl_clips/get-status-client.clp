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


(defrule cx-rl-get-status-selection-request
  (not (executive-fimalize))
  (cx-rl-node (name ?node))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/get_status"))) (type ?type))
  ?status-f <- (rl-get-status (node ?node) (request-id 0))
  (time ?now)
=>
  (bind ?new-req (ros-msgs-create-request ?type))
  (bind ?id (ros-msgs-async-send-request ?new-req ?s))
  (if ?id then
    (modify ?status-f (request-id ?id))
  )
  (ros-msgs-destroy-message ?new-req)
)

(defrule cx-rl-get-status-response-received
  ?node-f <- (cx-rl-node (name ?node))
  (ros-msgs-client (service ?s&:(eq ?s (str-cat ?node "/get_status"))) (type ?type))
  ?msg-fact <- (ros-msgs-response (service ?s) (msg-ptr ?ptr) (request-id ?id))
  ?req-meta <- (rl-get-status (node ?node) (request-id ?id))
=>
  (bind ?mode (sym-cat (ros-msgs-get-field ?ptr "mode")))
  (bind ?ep-count (ros-msgs-get-field ?ptr "current_episode"))
  (bind ?ep-step-count (ros-msgs-get-field ?ptr "current_episode_step"))
  (bind ?total-steps (ros-msgs-get-field ?ptr "current_episode_step"))
  (bind ?model-loaded (ros-msgs-get-field ?ptr "model_loaded"))

  (modify ?node-f (episode ?ep-count) (mode ?mode) (step ?ep-step-count) (total-steps ?total-steps) (model-loaded ?model-loaded))
  (ros-msgs-destroy-message ?ptr)
  (retract ?msg-fact)
  (retract ?req-meta)
)
