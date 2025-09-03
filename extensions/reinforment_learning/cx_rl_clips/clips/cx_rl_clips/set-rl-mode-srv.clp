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

(defrule set-rl-mode-service-init
" Create a service allowing clients to set the rl mode. "
    (not (cx-rl-interfaces-set-rl-mode-service (name "set_rl_mode")))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-set-rl-mode-create-service "set_rl_mode")
    (printout info "Created service for /set_rl_mode" crlf)
)

(deffunction cx-rl-interfaces-set-rl-mode-service-callback (?service-name ?request ?response)
    (bind ?mode (sym-cat (cx-rl-interfaces-set-rl-mode-request-get-field ?request "mode")))
    (printout info "Changing reinforcement learning mode to " ?mode crlf)
    (if (or (eq ?mode TRAINING) (eq ?mode EVALUATION) (eq ?mode EXECUTION)) then
        (assert (rl-mode (mode ?mode)))
        (cx-rl-interfaces-set-rl-mode-response-set-field ?response "confirmation" (str-cat "Set mode to " ?mode))
    else
        (cx-rl-interfaces-set-rl-mode-response-set-field ?response "confirmation" "Couldn't set mode")
    )
)
