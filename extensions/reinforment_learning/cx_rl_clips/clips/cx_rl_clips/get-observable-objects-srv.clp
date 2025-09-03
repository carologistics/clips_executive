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

(defrule get-observable-objects-service-init
" Create a service returning a list of all observable objects for a certain type in the clips environment."
    (not (cx-rl-interfaces-get-observable-objects-service (name "get_observable_objects")))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-get-observable-objects-create-service "get_observable_objects")
    (printout info "Created service for /get_observable_objects" crlf)
)

(deffunction cx-rl-interfaces-get-observable-objects-service-callback (?service-name ?request ?response)
    (bind ?t (cx-rl-interfaces-get-observable-objects-request-get-field ?request "type"))
    (printout info "Collecting cxrl observable objects of type " ?t crlf)
    (bind ?obj-list (create$))
    (do-for-all-facts ((?ot rl-observable-type))
            (eq ?ot:type (sym-cat ?t))
        (bind ?obj-list ?ot:objects)
        (break)
    )
    (if (eq ?obj-list (create$)) then
        (bind ?obj-list (insert$ ?obj-list 1 "Not found"))
    )

    (cx-rl-interfaces-get-observable-objects-response-set-field ?response "objects" ?obj-list)
)
