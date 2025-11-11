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

(deffunction create-action-param-string (?params)
    (bind ?size (length$ ?params))
    (bind ?param-string "")
    (loop-for-count (?i 1 ?size)
        (if (eq (mod ?i 2) 1) then
            (bind ?param-string (str-cat ?param-string (nth$ ?i ?params) "|"))
        else
            (bind ?param-string (str-cat ?param-string (nth$ ?i ?params) "#"))
        )
    )
    (return ?param-string)
)

(deffunction create-slot-value-string (?values)
    (bind ?size (length$ ?values))
    (bind ?value-string "")
    (loop-for-count (?i 1 ?size)
        (bind ?value-string (str-cat ?value-string (nth$ ?i ?values)))
        (if (neq ?i ?size) then
            (bind ?value-string (str-cat ?value-string "#"))
        )
    )
    (return ?value-string)
)

(deffunction rl-generate-observations () )

(deffunction rl-delete-observations ()
    (do-for-all-facts ((?ob rl-observation))
            TRUE
        (retract ?ob)
    )
)

(deffunction create-observation-string ()
    (printout info "Generating environment state string" crlf)
    (rl-generate-observations)
    (bind ?state-string "{")
    (do-for-all-facts ((?ob rl-observation))
            TRUE
        (bind ?fact-string (str-cat "\"" ?ob:name "(" (create-slot-value-string ?ob:param-values) ")\","))
        (bind ?state-string (str-cat ?state-string ?fact-string))
    )
    (bind ?state-string (str-cat (sub-string 1 (- (str-length ?state-string) 1) ?state-string) "}"))
    (rl-delete-observations)
    (return ?state-string)
)

(defrule all-services-actions-created
    (not (saved-facts))
	(cx-rl-node (name ?node))
    (ros-msgs-client (service ?s1&:(eq ?s1 (str-cat ?node "/exec_action_selection"))) (type ?type))
    (ros-msgs-service (service ?s2&:(eq ?s2 (str-cat ?node "/set_rl_mode"))) (type ?type))
    (ros-msgs-service (service ?s3&:(eq ?s3 (str-cat ?node "/create_rl_env_state"))) (type ?type))
    (ros-msgs-service (service ?s4&:(eq ?s4 (str-cat ?node "/get_observable_objects"))) (type ?type))
    (ros-msgs-service (service ?s5&:(eq ?s5 (str-cat ?node "/get_predefined_observables"))) (type ?type))
    (ros-msgs-service (service ?s6&:(eq ?s6 (str-cat ?node "/get_action_list_executable_for_robot"))) (type ?type))
    (ros-msgs-service (service ?s7&:(eq ?s7 (str-cat ?node "/get_action_list_executable"))) (type ?type))
    (cx-rl-interfaces-get-free-robot-server)
    (cx-rl-interfaces-action-selection-server)
    (cx-rl-interfaces-reset-cx-server)
=>
    (assert (saved-facts))
    (save-facts reset-save)
)
