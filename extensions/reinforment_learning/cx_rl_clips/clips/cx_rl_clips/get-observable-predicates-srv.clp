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

(defrule get-observable-predicates-service-init
" Create a service returning a list of all observable predicates in the clips environment."
    (not (cx-rl-interfaces-get-observable-predicates-service (name "get_observable_predicates")))
    (domain-facts-loaded)
=>
    (cx-rl-interfaces-get-observable-predicates-create-service "get_observable_predicates")
    (printout info "Created service for /get_observable_predicates" crlf)
)

(deffunction cx-rl-interfaces-get-observable-predicates-service-callback (?service-name ?request ?response)
    (printout info "Collecting cxrl observable predicates" crlf)
    (bind ?pred-names (create$))
    (bind ?param-counts (create$))
    (bind ?param-names (create$))
    (bind ?param-types (create$))

    (do-for-all-facts ((?dp rl-observable-predicate))
            TRUE
        (printout info "observable predicate " ?dp:name crlf)
        (bind ?pred-names (insert$ ?pred-names (+ (length$ ?pred-names) 1) (str-cat ?dp:name)))
        (bind ?param-counts (insert$ ?param-counts (+ (length$ ?param-counts) 1) (length$ ?dp:param-names)))
        (loop-for-count (?i 1 (length$ ?dp:param-names))
            (bind ?param-names (insert$ ?param-names (+ (length$ ?param-names) 1) (str-cat (nth$ ?i ?dp:param-names))))
            (bind ?param-types (insert$ ?param-types (+ (length$ ?param-types) 1) (str-cat (nth$ ?i ?dp:param-types))))
        )
    )

    (cx-rl-interfaces-get-observable-predicates-response-set-field ?response "predicatenames" ?pred-names)
    (cx-rl-interfaces-get-observable-predicates-response-set-field ?response "paramcounts" ?param-counts)
    (cx-rl-interfaces-get-observable-predicates-response-set-field ?response "paramnames" ?param-names)
    (cx-rl-interfaces-get-observable-predicates-response-set-field ?response "paramtypes" ?param-types)
)
