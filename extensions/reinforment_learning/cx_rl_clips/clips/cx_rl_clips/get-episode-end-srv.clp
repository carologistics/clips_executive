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
"(deffunction " ?*CX-RL-NODE-NAME* "/get_episode_end-service-callback (?service-name ?request ?response)
    (bind ?end FALSE)
    (bind ?reward 0)
    (do-for-fact ((?end-f rl-episode-end))
        (eq ?end-f:node \"" ?*CX-RL-NODE-NAME* "\")
        (bind ?end TRUE)
        (if ?end-f:success then
            (bind ?reward ?*REWARD-EPISODE-SUCCESS*)
          else
            (bind ?reward ?*REWARD-EPISODE-FAILURE*)
        )
    )
    (ros-msgs-set-field ?response \"episode_end\" ?end)
    (ros-msgs-set-field ?response \"reward\" ?reward)
)"
))
