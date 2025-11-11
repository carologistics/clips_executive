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

(build (str-cat
"(deffunction " ?*CX-RL-NODE-NAME* "/get_action_list_executable-service-callback (?service-name ?request ?response) \
    (printout info \"Generating list of all executable actions\" crlf) \
    (bind ?action-list (create$)) \
    (do-for-all-facts ((?action rl-action)) \
            (eq ?action:is-selected FALSE) \
        (bind ?action-string (str-cat ?action:id \"|\" ?action:name)) \
        (printout info "\Executable action: \" ?action-string crlf) \
        (bind ?action-list (insert$ ?action-list 1 ?action-string)) \
    ) \
    (ros-msgs-set-field ?response \"actions\" ?action-list) \
)"
))
