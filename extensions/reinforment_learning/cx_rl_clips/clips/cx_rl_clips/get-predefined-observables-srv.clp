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

(defrule get-predefined-observables-service-init
" Create a service returning a list of all observable predicates in the clips environment."
    (not (cx-rl-interfaces-get-predefined-observables-service (name "get_predefined_observables")))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-get-predefined-observables-create-service "get_predefined_observables")
    (printout info "Created service for /get_predefined_observables" crlf)
)

(deffunction cx-rl-interfaces-get-predefined-observables-service-callback (?service-name ?request ?response)
    (printout info "Collecting cxrl predefined observables" crlf)
    (bind ?observables (create$))

    (do-for-all-facts ((?po rl-predefined-observable))
            TRUE
        (printout info "predefined observable " ?po:name ?po:params crlf)
        (bind ?observables (insert$ ?observables (+ (length$ ?observables) 1) (str-cat ?po:name "(" (create-slot-value-string ?po:params) ")")))
    )

    (cx-rl-interfaces-get-predefined-observables-response-set-field ?response "observables" ?observables)
)
