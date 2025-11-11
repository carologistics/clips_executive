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

(build
(str-cat
"(deffunction " ?*CX-RL-NODE-NAME* "/get_predefined_observables-service-callback (?service-name ?request ?response)
    (printout info \"Collecting cx rl predefined observables\" crlf)
    (bind ?observables (create$))

    (do-for-all-facts ((?po rl-predefined-observable))
            TRUE
        (printout info "\predefined observable \" ?po:name ?po:params crlf)
        (bind ?observables (insert$ ?observables (+ (length$ ?observables) 1) (str-cat ?po:name \"(\" (create-slot-value-string ?po:params) \")\")))
    )

    (ros-msgs-set-field ?response \"observables\" ?observables)
)"
))
