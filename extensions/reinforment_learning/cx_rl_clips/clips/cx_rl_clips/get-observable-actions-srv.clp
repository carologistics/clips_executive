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
"(deffunction " ?*CX-RL-NODE-NAME* "/get_observable_actions-service-callback (?service-name ?request ?response)
    (printout ?*CX-RL-LOG-LEVEL* \"Collecting cxrl observable actions\" crlf)
    (bind ?action-names (create$))
    (bind ?param-counts (create$))
    (bind ?param-names (create$))
    (bind ?param-types (create$))

    (do-for-all-facts ((?oa rl-observable-action))
            (eq ?oa:node \"" ?*CX-RL-NODE-NAME* "\")
        (printout ?*CX-RL-LOG-LEVEL* \"observable action \" ?oa:name crlf)
        (bind ?action-names (insert$ ?action-names (+ (length$ ?action-names) 1) (str-cat ?oa:name)))
        (bind ?param-counts (insert$ ?param-counts (+ (length$ ?param-counts) 1) (length$ ?oa:param-names)))
        (loop-for-count (?i 1 (length$ ?oa:param-names))
            (bind ?param-names (insert$ ?param-names (+ (length$ ?param-names) 1) (str-cat (nth$ ?i ?oa:param-names))))
            (bind ?param-types (insert$ ?param-types (+ (length$ ?param-types) 1) (str-cat (nth$ ?i ?oa:param-types))))
        )
    )

    (ros-msgs-set-field ?response \"action_names\" ?action-names)
    (ros-msgs-set-field ?response \"param_counts\" ?param-counts)
    (ros-msgs-set-field ?response \"param_names\" ?param-names)
    (ros-msgs-set-field ?response \"param_types\" ?param-types)
)"))
