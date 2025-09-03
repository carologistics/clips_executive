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

(defrule create-rl-env-state-service-init
" Create a service allowing clients to recieve an observation of the current state of the environment. "
    (not (cx-rl-interfaces-create-rl-env-state-service (name "create_rl_env_state")))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-create-rl-env-state-create-service "create_rl_env_state")
    (printout info "Created service for /create_rl_env_state" crlf)
)



(deffunction cx-rl-interfaces-create-rl-env-state-service-callback (?service-name ?request ?response)
    (bind ?state-string (create-observation-string))
    (cx-rl-interfaces-create-rl-env-state-response-set-field ?response "state" ?state-string)
)
