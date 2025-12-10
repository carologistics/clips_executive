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
"(deffunction " ?*CX-RL-NODE-NAME* "/set_rl_mode-service-callback (?service-name ?request ?response)
    (bind ?mode (sym-cat (ros-msgs-get-field ?request \"mode\")))
    (printout ?*CX-RL-LOG-LEVEL* \"Changing reinforcement learning mode to \" ?mode crlf)
    (if (or (eq ?mode TRAINING) (eq ?mode EVALUATION) (eq ?mode EXECUTION)) then
        (if (not (do-for-fact ((?cx-rl cx-rl-node)) (eq ?cx-rl:name \"" ?*CX-RL-NODE-NAME* "\")
         (modify ?cx-rl (mode ?mode))
        )) then
        (ros-msgs-set-field ?response \"confirmation\" \"Couldn't set mode\")
        )
        (ros-msgs-set-field ?response \"confirmation\" (str-cat \"Set mode to \" ?mode))
    else
        (ros-msgs-set-field ?response \"confirmation\" \"Couldn't set mode\")
    )
)"
))
