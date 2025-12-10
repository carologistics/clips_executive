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

(deffunction cx-rl-create-slot-value-string (?values)
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

(deffunction cx-rl-create-observation-string (?node)
    (printout ?*CX-RL-LOG-LEVEL* "Generating environment state string" crlf)
    (bind ?state-string "{")
    (do-for-all-facts ((?ob rl-observation))
            (eq ?ob:node ?node)
        (bind ?fact-string (str-cat "\"" ?ob:name "(" (cx-rl-create-slot-value-string ?ob:param-values) ")\","))
        (bind ?state-string (str-cat ?state-string ?fact-string))
    )
    (bind ?state-string (str-cat (sub-string 1 (- (str-length ?state-string) 1) ?state-string) "}"))
    (return ?state-string)
)
